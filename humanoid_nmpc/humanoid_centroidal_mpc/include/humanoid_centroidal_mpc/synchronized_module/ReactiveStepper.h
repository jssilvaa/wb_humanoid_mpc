// Reactive gait injector + capture-point monitor for disturbance-driven stepping (B3).
//
// A SolverSynchronizedModule that owns the GaitSchedule and, when a new desired gait is staged,
// inserts it PROMPTLY at initTime + stepLeadTime. This bypasses GaitScheduleUpdater's 0.7*horizon
// "earliest switching time" heuristic, which is meant for smooth velocity-driven gait changes but
// defers a step by ~0.8 s -- far too slow for push recovery, where the foot must lift within tens
// of ms of the trigger.
//
// A single recovery step is two staged gaits: a swing-then-stand template (e.g. {LF, STANCE} =
// right foot swings, then both land) at the balance->stepping transition, then {STANCE} once the
// capture point is back inside support. Because insertModeSequenceTemplate makes the staged
// template the repeating tile, holding a swing template keeps the robot stepping until stance is
// requested -- the FSM gets multi-step recovery for free.
//
// Each preSolverRun the module also computes the capture point xi = c + cdot/omega from the MPC
// init state (CoM c via Pinocchio FK; cdot from the state's normalized linear centroidal momentum,
// = the ICPCost formula with the velocity term restored; omega = sqrt(g / c_z)), together with the
// two foot positions (the support polygon). This is the exact quantity the stepping trigger will
// test (rung 3); it is exposed via getCaptureState() so the harness can validate it against the
// MuJoCo-measured CP and characterise the support-polygon thresholds (rung 2).
//
// Thread-safe: setDesiredGait() stages the request under a mutex; the GaitSchedule mutation and the
// capture-state write happen in preSolverRun (MPC thread, no workers running), so both are
// consistent for the whole solve -- the same discipline as GaitScheduleUpdater /
// ExternalWrenchFeedforward. The gait change is read by the reference manager's modifyReferences on
// the NEXT solve (~12 ms at 80 Hz). The module is a dumb mechanism: the policy (when to step, which
// foot) lives in the caller -- scripted from the harness in rung 1, the capture-point FSM later.

#pragma once

#include <pinocchio/fwd.hpp>  // must precede any other pinocchio header

#include <cmath>
#include <memory>
#include <mutex>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/gait/GaitSchedule.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

class ReactiveStepper : public SolverSynchronizedModule {
 public:
  // Capture point + support geometry from one MPC init state, all world-frame.
  struct CaptureState {
    vector3_t com = vector3_t::Zero();
    vector2_t comVel = vector2_t::Zero();      // CoM horizontal velocity (= normalized linear momentum)
    vector2_t capturePoint = vector2_t::Zero();  // xi = c_xy + cdot_xy / omega
    vector3_t footL = vector3_t::Zero();       // contactNames[0] (LF) frame position
    vector3_t footR = vector3_t::Zero();       // contactNames[1] (RF) frame position
    scalar_t omega = 0.0;                      // sqrt(g / c_z)
    bool valid = false;
  };

  // stepLeadTime [s]: how far past initTime the inserted gait starts. A small lead keeps the
  // current node in its measured contact state and lets the switch land on a later node, which the
  // SQP handles more gracefully than an instantaneous contact change at node 0.
  ReactiveStepper(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                  const PinocchioInterface& pinocchioInterface,
                  const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                  const ModelSettings& modelSettings,
                  scalar_t stepLeadTime = 0.05)
      : gaitSchedulePtr_(std::move(gaitSchedulePtr)),
        pinocchioInterface_(pinocchioInterface),
        mpcRobotModelPtr_(mpcRobotModel.clone()),
        frameIdL_(pinocchioInterface_.getModel().getFrameId(modelSettings.contactNames[0])),
        frameIdR_(pinocchioInterface_.getModel().getFrameId(modelSettings.contactNames[1])),
        stepLeadTime_(stepLeadTime) {}

  // Stage a new gait. Applied on the next preSolverRun. Thread-safe. The caller is responsible for
  // only staging on an actual change (this module re-inserts whenever flagged).
  void setDesiredGait(const ModeSequenceTemplate& gait) {
    std::lock_guard<std::mutex> lock(mutex_);
    pendingGait_ = gait;
    gaitUpdated_ = true;
  }

  // Latest capture point + support geometry (thread-safe copy). Computed each preSolverRun.
  CaptureState getCaptureState() {
    std::lock_guard<std::mutex> lock(mutex_);
    return captureState_;
  }

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& currentState,
                    const ReferenceManagerInterface& /*referenceManager*/) override {
    const CaptureState cs = computeCaptureState(currentState);  // Pinocchio on our own data copy (this thread only)

    std::lock_guard<std::mutex> lock(mutex_);
    captureState_ = cs;
    if (gaitUpdated_) {
      // Tile from the prompt start out past the reference manager's read window (it reads
      // getModeSchedule up to finalTime + horizon); getModeSchedule re-tiles the template anyway.
      const scalar_t horizon = finalTime - initTime;
      gaitSchedulePtr_->insertModeSequenceTemplate(pendingGait_, initTime + stepLeadTime_, finalTime + horizon);
      gaitUpdated_ = false;
    }
  }

  void postSolverRun(const PrimalSolution&) override {}

 protected:
  CaptureState computeCaptureState(const vector_t& state) {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::centerOfMass(model, data, q, false);
    pinocchio::updateFramePlacement(model, data, frameIdL_);
    pinocchio::updateFramePlacement(model, data, frameIdR_);

    CaptureState cs;
    cs.com = data.com[0];
    cs.comVel = state.head<2>();  // normalized linear centroidal momentum = CoM velocity (world x, y)
    cs.omega = std::sqrt(gravity_ / std::max(cs.com.z(), scalar_t(1e-3)));
    cs.capturePoint = cs.com.head<2>() + cs.comVel / cs.omega;
    cs.footL = data.oMf[frameIdL_].translation();
    cs.footR = data.oMf[frameIdR_].translation();
    cs.valid = true;
    return cs;
  }

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  PinocchioInterface pinocchioInterface_;  // mutable copy (owns its own data for the FK above)
  std::unique_ptr<MpcRobotModelBase<scalar_t>> mpcRobotModelPtr_;
  size_t frameIdL_;
  size_t frameIdR_;
  scalar_t gravity_{9.81};
  scalar_t stepLeadTime_;

  std::mutex mutex_;
  bool gaitUpdated_{false};
  ModeSequenceTemplate pendingGait_{{0.0, 0.5}, {ModeNumber::STANCE}};
  CaptureState captureState_;
};

}  // namespace ocs2::humanoid
