// Shared buffer carrying the external-wrench feedforward into the centroidal dynamics (ADR / B2).
//
// CentroidalDynamicsAD adds  decay * wrench / mass  to the normalized centroidal-momentum rate,
// where decay = exp(-(node_time - t0) / T) is the assumed disturbance persistence over the horizon
// (the optimal forward prediction of a first-order / Ornstein-Uhlenbeck disturbance with correlation
// time T). invT = 1/T; invT = 0 is the zero-order-hold limit (infinite persistence). The decay
// depends only on the node time, so the dynamics term stays affine in (x, u) -- no AD re-trace.
//
// Written once per solve by ExternalWrenchFeedforward::preSolverRun (single-threaded safe point);
// read by the cloned dynamics across solver threads during the solve.

#pragma once

#include <ocs2_core/Types.h>

namespace ocs2::humanoid {

struct ExternalWrenchBuffer {
  vector_t wrench = vector_t::Zero(6);  // W_hat = [f; tau] about CoM, world frame (already trust-scaled)
  scalar_t t0 = 0.0;                    // horizon start time (set to initTime each solve)
  scalar_t invT = 0.0;                  // 1/T horizon decay rate; 0 => no decay (ZOH, infinite persistence)
};

}  // namespace ocs2::humanoid
