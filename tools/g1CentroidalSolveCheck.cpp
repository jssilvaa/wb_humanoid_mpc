// B1 kickoff / Phase 0b definitive proof: instantiate the REAL G1 centroidal MPC
// (from the same config the ROS2 node uses) and run ONE SQP solve, standing in
// place (zero commanded velocity). If this passes, the standalone substrate is
// proven usable: URDF load -> Pinocchio centroidal model -> CppAD codegen -> OCP
// assembly -> SQP/HPIPM solve all work without ROS2.
//
// Mirrors CentroidalMpcSqpNode.cpp's wiring, but with the non-ROS2 base classes
// (ProceduralMpcMotionManager instead of its Ros2 subclass) and a direct mpc.run()
// instead of MPC_ROS_Interface.
//
// First run triggers G1 CppAD codegen (~5-15 min) into ./build/<modelFolderCppAd>
// (relative to CWD; task.info sets recompileLibrariesCppAd=false, so it caches).
// The centroidal model type (Full=0 / SRBD=1) is task.info:centroidalModelType --
// the B1 ablation variable.

#include <cmath>
#include <cstdlib>
#include <iostream>

#include <ocs2_sqp/SqpMpc.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
  // Config paths: compile-time defaults (source tree), overridable by argv.
  const std::string taskFile = (argc > 1) ? argv[1] : G1_TASK_FILE;
  const std::string urdfFile = (argc > 2) ? argv[2] : G1_URDF_FILE;
  const std::string referenceFile = (argc > 3) ? argv[3] : G1_REFERENCE_FILE;
  const std::string gaitFile = (argc > 4) ? argv[4] : G1_GAIT_FILE;

  std::cout << "=== G1 centroidal-MPC standalone solve check ===\n";
  std::cout << "task : " << taskFile << "\nurdf : " << urdfFile << "\n";

  // 1. Robot interface (URDF -> centroidal model -> OCP; CppAD codegen on first run).
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile, true);

  // 2. SQP MPC over the assembled OCP.
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 3. Reference + procedural motion management (stand: zero commanded velocity).
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});  // default = stand (v=0, h=0.8)

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(motionManager);

  // 4. One MPC solve from the nominal initial state.
  const scalar_t t0 = 0.0;
  const vector_t x0 = interface.getInitialState();
  std::cout << "state dim: " << x0.size() << "   horizon: " << interface.mpcSettings().timeHorizon_ << " s\n";

  // Seed an initial standing target: the motion manager refreshes TargetTrajectories
  // in its preSolverRun, but that runs AFTER the reference manager is first consulted,
  // so the very first solve needs a non-empty seed (mirrors the MRT publishing one).
  interface.getSwitchedModelReferenceManagerPtr()->setTargetTrajectories(
      targetCalc.commandedVelocityToTargetTrajectories(vector4_t::Zero(), t0, x0));

  const bool ok = mpc.run(t0, x0);
  const scalar_t tf = t0 + interface.mpcSettings().timeHorizon_;
  const auto sol = mpc.getSolverPtr()->primalSolution(tf);
  const auto perf = mpc.getSolverPtr()->getPerformanceIndeces();

  std::cout << "--- result ---\n";
  std::cout << "  mpc.run() returned     : " << (ok ? "true" : "false") << "\n";
  std::cout << "  horizon nodes          : " << sol.timeTrajectory_.size() << "\n";
  std::cout << "  cost                   : " << perf.cost << "\n";
  std::cout << "  dynamicsViolationSSE   : " << perf.dynamicsViolationSSE << "\n";
  std::cout << "  equalityConstraintsSSE : " << perf.equalityConstraintsSSE << "\n";

  // Smoke-test criteria: it solved without blowing up. Tight convergence tolerances
  // are a B1 tuning concern; here we assert structural sanity + finiteness.
  int failures = 0;
  auto check = [&](bool c, const std::string& w) {
    std::cout << (c ? "  [PASS] " : "  [FAIL] ") << w << "\n";
    if (!c) {
      ++failures;
    }
  };
  check(ok, "mpc.run() succeeded");
  check(sol.timeTrajectory_.size() > 1, "non-trivial horizon produced");
  check(sol.stateTrajectory_.size() == sol.timeTrajectory_.size(), "state trajectory sized to horizon");
  check(!sol.stateTrajectory_.empty() && sol.stateTrajectory_.front().isApprox(x0, 1e-6), "initial state preserved");
  check(std::isfinite(perf.cost), "cost is finite");
  check(std::isfinite(perf.dynamicsViolationSSE), "dynamics violation is finite");
  check(std::isfinite(perf.equalityConstraintsSSE), "equality-constraint violation is finite");

  std::cout << (failures == 0 ? "=== G1 SOLVE PASS ===\n" : "=== G1 SOLVE FAIL ===\n");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
