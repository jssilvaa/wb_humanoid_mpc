// Phase 0b solve milestone: prove the standalone OCS2 SQP stack RUNS end-to-end
// on macOS/clang (not just compiles). This is the real test of the BLASFEO/HPIPM
// linkage: a tag mismatch in hpipm (e.g. d_ocp_qp_dim_set_all) only surfaces when
// an executable actually links + calls into HPIPM, which this does.
//
// It solves OCS2's canonical circular-kinematics OCP (a particle constrained to a
// unit circle, asked to orbit at 1 m/s): exercises CppAD/CppADCodeGen codegen, the
// SQP multiple-shooting loop, the HPIPM QP backend, and state-input eq projection.
// Mirrors testCircularKinematics.cpp but with plain checks (no gtest dependency).

#include <cstdlib>
#include <iostream>

#include "ocs2_sqp/SqpSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/test/circular_kinematics.h>

namespace {
int failures = 0;
void check(bool cond, const std::string& what) {
  std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << what << '\n';
  if (!cond) {
    ++failures;
  }
}
}  // namespace

int main() {
  std::cout << "=== OCS2 standalone SQP solve milestone (circular kinematics) ===\n";

  // Optimal control problem (triggers CppAD codegen into /tmp on first run).
  ocs2::OptimalControlProblem problem = ocs2::createCircularKinematicsProblem("/tmp/sqp_milestone_generated");

  ocs2::DefaultInitializer zeroInitializer(2);

  ocs2::sqp::Settings settings;
  settings.dt = 0.01;
  settings.sqpIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = false;
  settings.printLinesearch = false;
  settings.nThreads = 1;

  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  ocs2::SqpSolver solver(settings, problem, zeroInitializer);
  solver.run(startTime, initState, 0, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);
  const auto performance = solver.getPerformanceIndeces();

  std::cout << "--- result ---\n";
  std::cout << "  horizon nodes : " << primalSolution.timeTrajectory_.size() << '\n';
  std::cout << "  dynamicsViolationSSE   : " << performance.dynamicsViolationSSE << '\n';
  std::cout << "  equalityConstraintsSSE : " << performance.equalityConstraintsSSE << '\n';

  // Convergence + structural checks (same tolerances as OCS2's own gtest).
  check(primalSolution.stateTrajectory_.front().isApprox(initState), "initial state preserved");
  check(primalSolution.timeTrajectory_.front() == startTime, "start time correct");
  check(primalSolution.timeTrajectory_.back() == finalTime, "final time correct");
  check(performance.dynamicsViolationSSE < 1e-6, "dynamics violation < 1e-6");
  check(performance.equalityConstraintsSSE < 1e-6, "equality-constraint violation < 1e-6");

  std::cout << (failures == 0 ? "=== MILESTONE PASS ===\n" : "=== MILESTONE FAIL ===\n");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
