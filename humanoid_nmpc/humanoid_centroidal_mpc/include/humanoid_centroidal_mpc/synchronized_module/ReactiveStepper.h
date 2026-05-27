// Reactive gait injector for disturbance-driven stepping (active disturbance rejection, B3).
//
// A SolverSynchronizedModule that owns the GaitSchedule and, when a new desired gait is staged,
// inserts it PROMPTLY at initTime + stepLeadTime. This bypasses GaitScheduleUpdater's
// 0.7*horizon "earliest switching time" heuristic, which is meant for smooth velocity-driven gait
// changes but defers a step by ~0.8 s -- far too slow for push recovery, where the foot must lift
// within tens of ms of the trigger.
//
// A single recovery step is two staged gaits: a swing template (e.g. {LF} = right foot swings) at
// the balance->stepping transition, then {STANCE} once the capture point is back inside support
// (stepping->balance). Because insertModeSequenceTemplate makes the staged template the repeating
// tile, holding the swing template keeps the robot stepping until stance is requested -- the FSM
// gets multi-step recovery for free.
//
// Thread-safe: setDesiredGait() stages the request under a mutex; the GaitSchedule mutation happens
// in preSolverRun (MPC thread, no workers running), so the schedule is consistent for the whole
// solve -- the same discipline as GaitScheduleUpdater and ExternalWrenchFeedforward. The change is
// read by the reference manager's modifyReferences on the NEXT solve (~12 ms at 80 Hz).
//
// The module is deliberately a dumb mechanism: it inserts whatever gait it is handed. The policy
// (when to step, which foot) lives in the caller -- a scripted single step from the harness in the
// footstep-policy verification (rung 1), and the capture-point FSM in the later rungs.

#pragma once

#include <memory>
#include <mutex>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "humanoid_common_mpc/gait/GaitSchedule.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

class ReactiveStepper : public SolverSynchronizedModule {
 public:
  // stepLeadTime [s]: how far past initTime the inserted gait starts. A small lead keeps the
  // current node in its measured contact state and lets the switch land on a later node, which the
  // SQP handles more gracefully than an instantaneous contact change at node 0.
  ReactiveStepper(std::shared_ptr<GaitSchedule> gaitSchedulePtr, scalar_t stepLeadTime = 0.05)
      : gaitSchedulePtr_(std::move(gaitSchedulePtr)), stepLeadTime_(stepLeadTime) {}

  // Stage a new gait. Applied on the next preSolverRun. Thread-safe. The caller is responsible for
  // only staging on an actual change (this module re-inserts whenever flagged).
  void setDesiredGait(const ModeSequenceTemplate& gait) {
    std::lock_guard<std::mutex> lock(mutex_);
    pendingGait_ = gait;
    gaitUpdated_ = true;
  }

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& /*currentState*/,
                    const ReferenceManagerInterface& /*referenceManager*/) override {
    std::lock_guard<std::mutex> lock(mutex_);
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
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  scalar_t stepLeadTime_;
  std::mutex mutex_;
  bool gaitUpdated_{false};
  ModeSequenceTemplate pendingGait_{{0.0, 0.5}, {ModeNumber::STANCE}};
};

}  // namespace ocs2::humanoid
