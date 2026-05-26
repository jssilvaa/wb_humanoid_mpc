// B1 closed-loop harness (standing, no push) -- the first closed-loop milestone
// of the ViC track. It is a non-ROS2 version of
//   humanoid_centroidal_mpc_ros2/src/CentroidalMpcRobotSim.cpp
// minus rclcpp and the RViz visualizer: it drives the REAL G1 centroidal MPC in
// closed loop on the MuJoCo G1 through CentroidalMpcMrtJointController (which maps
// the MPC policy to joint torques via inverse dynamics), and checks the robot
// stays upright. Runs HEADLESS to sidestep the macOS "GLFW window must own the
// main thread" constraint (config.headless=true makes startSim() spawn only the
// physics thread and return).
//
// This verifies the de-ROS'd MPC + MRT stack actually STABILIZES the robot in
// closed loop. The xfrc push + CoM/DCM logging and the Full-vs-SRBD comparison
// build on top of this once standing is confirmed.
//
// Threading mirrors the ROS2 sim: physics runs in MujocoSimInterface's own thread,
// the MPC solve runs in the controller's solver thread, and this main loop is the
// ~500 Hz MRT control loop (read state -> computeJointControlAction -> apply).
//
// sim_seconds is REAL (wall-clock) time: the sim thread runs in real time, so the
// loop runs for that many real seconds. Config paths are compile-time defaults
// (source tree), all argv-overridable:
//   standingClosedLoop [sim_seconds] [task] [urdf] [reference] [gait] [scene]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_sqp/SqpMpc.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <robot_model/RobotDescription.h>
#include <robot_model/RobotState.h>

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
  const double sim_T = (argc > 1) ? std::stod(argv[1]) : 5.0;  // real (wall-clock) seconds
  const std::string taskFile = (argc > 2) ? argv[2] : G1_TASK_FILE;
  const std::string urdfFile = (argc > 3) ? argv[3] : G1_URDF_FILE;
  const std::string referenceFile = (argc > 4) ? argv[4] : G1_REFERENCE_FILE;
  const std::string gaitFile = (argc > 5) ? argv[5] : G1_GAIT_FILE;
  const std::string sceneFile = (argc > 6) ? argv[6] : G1_SCENE_FILE;

  std::cout << "=== G1 centroidal-MPC standing closed-loop (headless) ===\n";
  std::cout << "task  : " << taskFile << "\nscene : " << sceneFile << "\nsim_T : " << sim_T << " s\n";

  // 1. Interface (URDF -> centroidal model -> OCP; CppAD codegen on first run per type) + SQP MPC.
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 2. Procedural motion manager for the stance gait/mode schedule (zero velocity = stand).
  //    Mirrors CentroidalMpcRobotSim, with the non-ROS2 base ProceduralMpcMotionManager.
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});  // default = stand

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(motionManager);
  // No explicit setTargetTrajectories seed needed: the MRT controller's solverWorker
  // calls resetMpcNode(currentObservationToResetTrajectory(...)) before its first
  // advanceMpc, which sets the initial standing target on this reference manager.

  // 3. Sim initial state = the MPC nominal initial state (mirror CentroidalMpcRobotSim).
  robot::model::RobotDescription robotDescription(urdfFile);
  robot::model::RobotState initState(robotDescription, 2);
  initState.setConfigurationToZero();
  const vector_t& initMpcState = interface.getInitialState();
  const auto& mpcModel = interface.getMpcRobotModel();
  initState.setRootPositionInWorldFrame(mpcModel.getBasePosition(initMpcState));
  const vector_t mpcJointAngles = mpcModel.getJointAngles(initMpcState);
  const std::vector<robot::joint_index_t> mpcJointIndices =
      robotDescription.getJointIndices(interface.modelSettings().mpcModelJointNames);
  for (size_t i = 0; i < mpcJointIndices.size(); i++) {
    initState.setJointPosition(mpcJointIndices[i], mpcJointAngles[i]);
  }
  std::cout << "init base pos: " << initState.getRootPositionInWorldFrame().transpose() << "\n";

  // 4. MuJoCo sim interface (HEADLESS: physics thread only, no renderer/main-thread block).
  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = sceneFile;
  config.headless = true;
  config.verbose = false;
  config.initStatePtr_ = std::make_shared<robot::model::RobotState>(std::move(initState));
  robot::mujoco_sim_interface::MujocoSimInterface robotInterface(config, urdfFile);

  // 5. MRT joint controller (MPC policy -> joint torques). No visualizer in standalone.
  CentroidalMpcMrtJointController ctrl(robotInterface.getRobotDescription(), interface.modelSettings(), interface.getMpcRobotModel(), mpc,
                                       interface.getPinocchioInterface(), interface.mpcSettings().mpcDesiredFrequency_);

  // 6. Start sim + MPC thread; wait for the first policy (bounded, so a stuck solve fails fast).
  const size_t mrtDeltaTMicroSeconds = 1000000 / 500;  // 500 Hz control loop
  robotInterface.initSim();
  robotInterface.updateInterfaceStateFromRobot();
  ctrl.startMpcThread(robotInterface.getRobotState());

  const auto readyDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (!ctrl.ready()) {
    if (std::chrono::steady_clock::now() > readyDeadline) {
      std::cerr << "[standingClosedLoop] FAIL: no initial MPC policy within 60 s\n";
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Initial MPC policy received. Starting sim.\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // let the policy settle
  robotInterface.startSim();

  // 7. Control loop, WALL-CLOCK bounded: the sim thread advances physics in real time,
  //    so we run the MRT loop for sim_T real seconds (pacing to <=500 Hz; if a control
  //    step overruns the 2 ms budget the loop just runs as fast as it can). Track base
  //    height; stop early on a clear fall.
  double min_z = 1e9, last_z = 0.0;
  long iters = 0;
  bool fell = false;
  const auto loopStart = std::chrono::steady_clock::now();
  const auto loopEnd = loopStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim_T));
  auto nextLog = loopStart;
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= loopEnd) break;
    const auto targetTime = now + std::chrono::microseconds(mrtDeltaTMicroSeconds);

    robotInterface.updateInterfaceStateFromRobot();
    const robot::model::RobotState& st = robotInterface.getRobotState();
    ctrl.computeJointControlAction(0.0, st, robotInterface.getRobotJointAction());
    robotInterface.applyJointAction();
    ++iters;

    last_z = st.getRootPositionInWorldFrame().z();
    min_z = std::min(min_z, last_z);
    const double tw = std::chrono::duration<double>(now - loopStart).count();
    if (now >= nextLog) {
      std::cout << "  t=" << tw << " s   base z=" << last_z << " m" << std::endl;  // endl: flush to watch live
      nextLog = now + std::chrono::milliseconds(500);
    }
    if (last_z < 0.3) {  // clearly toppled -- stop early
      fell = true;
      std::cerr << "  fell at t=" << tw << " s (base z=" << last_z << " m)" << std::endl;
      break;
    }
    std::this_thread::sleep_until(targetTime);
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - loopStart).count();

  // 8. Report. Nominal standing base height ~0.79 m; >0.5 m = "did not fall" (matches observerHarness).
  std::cout << "--- result ---\n";
  std::cout << "  control steps : " << iters << " over " << wall << " s wall  -> " << (iters / wall)
            << " Hz achieved (target 500 Hz)\n";
  std::cout << "  final base z  : " << last_z << " m   min base z = " << min_z << " m\n";
  const bool stood = !fell && min_z > 0.5;
  std::cout << (stood ? "[standingClosedLoop] PASS: G1 stands under closed-loop MPC\n"
                      : "[standingClosedLoop] FAIL: G1 did not stay upright\n");

  // Hard-exit: CentroidalMpcMrtJointController's solverWorker() is a while(true) that
  // never checks terminateThread_, so its dtor blocks forever on solver_worker_.join()
  // (the ROS2 sim sidesteps this by never returning from main). Flush + _Exit abandons
  // the sim/solver threads cleanly for a one-shot eval; one process per ablation arm.
  std::cout.flush();
  std::cerr.flush();
  std::_Exit(stood ? 0 : 2);
}
