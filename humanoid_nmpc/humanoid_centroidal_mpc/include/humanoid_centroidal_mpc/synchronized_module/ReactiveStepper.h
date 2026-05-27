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
    bool outsideSupport = false;               // capture point outside the (margin-shrunk) support box
    int fsmState = 0;                          // 0 = balance, 1 = stepping (auto mode)
    int stepCount = 0;                         // number of steps staged so far (auto mode)
    bool valid = false;
  };

  // Foot half-extents [m] (G1: x_front 0.12, x_back 0.05, y 0.03 per report section 07) + the
  // capture-point trigger knobs. The support box is the bounding box of the contact feet grown by
  // these extents and shrunk by margin; CP outside it triggers a step. settleVel is the CoM-speed
  // below which "stepping -> balance" may complete; relandGap delays re-evaluation until a staged
  // step has landed (swingDur + relandGap after staging).
  struct StepParams {
    scalar_t xFront = 0.12;
    scalar_t xBack = 0.05;
    scalar_t yHalf = 0.03;
    scalar_t margin = 0.02;
    scalar_t swingDur = 0.5;
    scalar_t settleVel = 0.15;
    scalar_t relandGap = 0.1;
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

  // Enable the capture-point stepping FSM (rung 3). Without this the module only computes the CP and
  // applies manually-staged gaits (setDesiredGait, used by the rung-1 scripted probe).
  void enableAutoStepping() { enableAutoStepping(StepParams{}); }
  void enableAutoStepping(const StepParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    autoEnabled_ = true;
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
    CaptureState cs = computeCaptureState(currentState);  // Pinocchio on our own data copy (this thread only)

    std::lock_guard<std::mutex> lock(mutex_);
    if (autoEnabled_) runStepFsm(initTime, cs);  // capture-point FSM: may stage a step/stance
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

  // Support box (world xy) = bounding box of the two contact feet grown by the foot half-extents and
  // shrunk by margin. Returns whether the CP is outside and, if so, the swing mode for a recovery
  // step: foot toward the dominant exit edge (CP off +y/left -> swing LEFT foot = RF; off -y/right ->
  // swing RIGHT = LF); a sagittal exit defaults to a right-foot swing, the free foothold sets fwd/back.
  bool decideStep(const CaptureState& cs, size_t& swingModeOut) const {
    const scalar_t xMin = std::min(cs.footL.x(), cs.footR.x()) - params_.xBack + params_.margin;
    const scalar_t xMax = std::max(cs.footL.x(), cs.footR.x()) + params_.xFront - params_.margin;
    const scalar_t yMin = std::min(cs.footL.y(), cs.footR.y()) - params_.yHalf + params_.margin;
    const scalar_t yMax = std::max(cs.footL.y(), cs.footR.y()) + params_.yHalf - params_.margin;
    const scalar_t cx = cs.capturePoint.x(), cy = cs.capturePoint.y();
    const scalar_t exFwd = cx - xMax, exBack = xMin - cx, exLeft = cy - yMax, exRight = yMin - cy;
    const scalar_t maxEx = std::max(std::max(exFwd, exBack), std::max(exLeft, exRight));
    if (maxEx <= 0.0) return false;  // CP inside the (shrunk) support box
    if (maxEx == exLeft) {
      swingModeOut = ModeNumber::RF;
    } else if (maxEx == exRight) {
      swingModeOut = ModeNumber::LF;
    } else {
      swingModeOut = ModeNumber::LF;  // sagittal exit
    }
    return true;
  }

  ModeSequenceTemplate buildStepGait(size_t swingMode) const {
    return ModeSequenceTemplate({0.0, params_.swingDur, params_.swingDur + 100.0}, {swingMode, ModeNumber::STANCE});
  }

  // Capture-point FSM (call under lock): stage a step on balance->stepping, another step if the CP
  // is still outside after the previous one lands (adaptive multi-step), or stance once recovered.
  void runStepFsm(scalar_t initTime, CaptureState& cs) {
    size_t swingMode = ModeNumber::LF;
    const bool outside = decideStep(cs, swingMode);
    cs.outsideSupport = outside;
    const bool settled = cs.comVel.norm() < params_.settleVel;

    if (stepPhase_ == StepPhase::Balance) {
      if (outside) {
        pendingGait_ = buildStepGait(swingMode);
        gaitUpdated_ = true;
        stepPhase_ = StepPhase::Stepping;
        lastStageTime_ = initTime;
        ++stepCount_;
      }
    } else if (initTime > lastStageTime_ + params_.swingDur + params_.relandGap) {  // step has landed: re-evaluate
      if (outside) {
        pendingGait_ = buildStepGait(swingMode);  // still off support -> another step toward the current CP
        gaitUpdated_ = true;
        lastStageTime_ = initTime;
        ++stepCount_;
      } else if (settled) {
        pendingGait_ = ModeSequenceTemplate({0.0, 0.5}, {ModeNumber::STANCE});  // recovered -> hold stance
        gaitUpdated_ = true;
        stepPhase_ = StepPhase::Balance;
      }
    }
    cs.fsmState = (stepPhase_ == StepPhase::Stepping) ? 1 : 0;
    cs.stepCount = stepCount_;
  }

  enum class StepPhase { Balance, Stepping };

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

  // Capture-point stepping FSM (auto mode).
  bool autoEnabled_{false};
  StepParams params_;
  StepPhase stepPhase_{StepPhase::Balance};
  scalar_t lastStageTime_{-1e9};
  int stepCount_{0};
};

}  // namespace ocs2::humanoid
