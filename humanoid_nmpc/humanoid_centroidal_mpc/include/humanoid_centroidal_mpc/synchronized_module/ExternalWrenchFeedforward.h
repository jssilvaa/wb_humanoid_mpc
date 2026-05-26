// External-wrench feedforward for the centroidal MPC (active disturbance rejection, B2).
//
// A SolverSynchronizedModule that, once per solve (preSolverRun), freezes the latest external-wrench
// estimate W_hat = [f; tau] (linear; angular, about the CoM, world frame) into the shared
// ExternalWrenchBuffer the centroidal dynamics read. Freezing once per solve gives every node and
// every solver worker thread a consistent wrench over the horizon and keeps the SQP linearization
// self-consistent. The dynamics add decay*W_hat/mass to the normalized centroidal-momentum rate,
// with decay = exp(-(node_time - t0)/T) modelling the assumed disturbance persistence; because the
// wrench + decay are exogenous (independent of state/input) the term is affine -> no AD re-trace.
//
// Horizon decay T: persistence over a single horizon. NOT the same as the observer bandwidth (how
// fast W_hat tracks the disturbance) -- T governs forward extrapolation only. True persistence is
// handled by re-injection across solves (~80 Hz), so T should be short; T = inf is the ZOH limit.
//
// setWrench() is the thread-safe source input: the harness feeds the true scripted wrench in the
// oracle phase (B2.0); the centroidal-momentum observer feeds its estimate in B2.1.

#pragma once

#include <cmath>
#include <memory>
#include <mutex>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "humanoid_centroidal_mpc/dynamics/ExternalWrenchBuffer.h"

namespace ocs2::humanoid {

class ExternalWrenchFeedforward final : public SolverSynchronizedModule {
 public:
  // decayTime T [s]: factor = exp(-s/T) over the horizon. T <= 0 or +inf => no decay (ZOH).
  // trust in [0, 1] scales the fed-forward wrench to hedge against estimator error.
  ExternalWrenchFeedforward(std::shared_ptr<ExternalWrenchBuffer> buffer, scalar_t decayTime, scalar_t trust = 1.0)
      : buffer_(std::move(buffer)),
        invT_((decayTime > 0.0 && std::isfinite(decayTime)) ? 1.0 / decayTime : 0.0),
        trust_(trust),
        liveWrench_(vector_t::Zero(6)) {}

  // Thread-safe; called by the source (oracle harness in B2.0, observer in B2.1).
  void setWrench(const vector_t& wrench) {
    std::lock_guard<std::mutex> lock(mutex_);
    liveWrench_ = wrench;
  }

  // Freeze the current estimate for the upcoming solve. Called before the solve on the MPC thread,
  // with no worker threads running, so writing the shared buffer here is safe against the
  // (subsequent, read-only) dynamics evaluations.
  void preSolverRun(scalar_t initTime, scalar_t, const vector_t&, const ReferenceManagerInterface&) override {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_->wrench = trust_ * liveWrench_;
    buffer_->t0 = initTime;
    buffer_->invT = invT_;
  }

  void postSolverRun(const PrimalSolution&) override {}

 private:
  std::shared_ptr<ExternalWrenchBuffer> buffer_;  // shared with the (cloned) centroidal dynamics
  scalar_t invT_;
  scalar_t trust_;
  vector_t liveWrench_;
  std::mutex mutex_;
};

}  // namespace ocs2::humanoid
