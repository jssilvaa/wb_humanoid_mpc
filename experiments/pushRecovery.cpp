// B1 push-recovery harness (balance regime, no stepping) -- builds on the verified
// standingClosedLoop by adding a scripted xfrc disturbance at torso_link and
// measuring whether the closed-loop centroidal MPC rejects it. Same wiring as
// standingClosedLoop (headless MujocoSimInterface + CentroidalMpcMrtJointController);
// the only addition is the push (MujocoSimInterface::setExternalWrench) + recovery
// metrics.
//
// This is STEP 2a: verify the push perturbs the closed loop and the MPC recovers,
// using base-pose metrics only. STEP 2b adds the per-step CoM/DCM/momentum CSV
// (mj_subtreeVel, world-frame, report convention) and the Full-vs-SRBD ablation.
//
// Default push = minor_sagittal (+x 100 N / 0.1 s) from report section 07 -- below the
// stepping threshold, so the robot should recover in place. argv:
//   pushRecovery [sim_sec=6] [fx=100] [fy=0] [t_push=2.0] [dur=0.1]
// (real wall-clock seconds; the sim thread runs in real time.)

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
  const double sim_T = (argc > 1) ? std::stod(argv[1]) : 6.0;    // real (wall-clock) seconds
  const double fx = (argc > 2) ? std::stod(argv[2]) : 100.0;     // push force x [N]
  const double fy = (argc > 3) ? std::stod(argv[3]) : 0.0;       // push force y [N]
  const double t_push = (argc > 4) ? std::stod(argv[4]) : 2.0;   // push onset [s]
  const double dur = (argc > 5) ? std::stod(argv[5]) : 0.1;      // push duration [s]
  const std::string taskFile = G1_TASK_FILE;
  const std::string urdfFile = G1_URDF_FILE;
  const std::string referenceFile = G1_REFERENCE_FILE;
  const std::string gaitFile = G1_GAIT_FILE;
  const std::string sceneFile = G1_SCENE_FILE;

  std::cout << "=== G1 centroidal-MPC push recovery (headless) ===\n";
  std::cout << "push: f=(" << fx << ", " << fy << ", 0) N at torso_link, t=[" << t_push << ", " << (t_push + dur)
            << "] s   sim_T=" << sim_T << " s\n";

  // 1. Interface + SQP MPC.
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 2. Stance gait/mode schedule (zero commanded velocity).
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(motionManager);

  // 3. Sim initial state = MPC nominal initial state.
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

  // 4. Headless MuJoCo sim.
  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = sceneFile;
  config.headless = true;
  config.verbose = false;
  config.initStatePtr_ = std::make_shared<robot::model::RobotState>(std::move(initState));
  robot::mujoco_sim_interface::MujocoSimInterface robotInterface(config, urdfFile);

  // 5. MRT joint controller.
  CentroidalMpcMrtJointController ctrl(robotInterface.getRobotDescription(), interface.modelSettings(), interface.getMpcRobotModel(), mpc,
                                       interface.getPinocchioInterface(), interface.mpcSettings().mpcDesiredFrequency_);

  // 6. Start sim + MPC thread.
  const size_t mrtDeltaTMicroSeconds = 1000000 / 500;
  robotInterface.initSim();
  robotInterface.updateInterfaceStateFromRobot();
  ctrl.startMpcThread(robotInterface.getRobotState());
  const auto readyDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (!ctrl.ready()) {
    if (std::chrono::steady_clock::now() > readyDeadline) {
      std::cerr << "[pushRecovery] FAIL: no initial MPC policy within 60 s\n";
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  robotInterface.startSim();

  // 7. Control loop, wall-clock bounded. Apply the push during its window; track the
  //    base-pose response. baseline xy is sampled just before push onset.
  const vector3_t pushForce(fx, fy, 0.0);
  double min_z = 1e9, last_z = 0.0, peak_disp = 0.0;
  double base_x0 = 0.0, base_y0 = 0.0;
  bool baselineSet = false, pushApplied = false, pushCleared = false, fell = false;
  long iters = 0;
  const auto loopStart = std::chrono::steady_clock::now();
  const auto loopEnd = loopStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim_T));
  auto nextLog = loopStart;
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= loopEnd) break;
    const double t = std::chrono::duration<double>(now - loopStart).count();
    const auto targetTime = now + std::chrono::microseconds(mrtDeltaTMicroSeconds);

    robotInterface.updateInterfaceStateFromRobot();
    const robot::model::RobotState& st = robotInterface.getRobotState();
    ctrl.computeJointControlAction(0.0, st, robotInterface.getRobotJointAction());
    robotInterface.applyJointAction();
    ++iters;

    // Push schedule (idempotent edges).
    if (!baselineSet && t >= t_push - 0.01) {
      base_x0 = st.getRootPositionInWorldFrame().x();
      base_y0 = st.getRootPositionInWorldFrame().y();
      baselineSet = true;
    }
    if (!pushApplied && t >= t_push) {
      robotInterface.setExternalWrench("torso_link", pushForce);
      pushApplied = true;
      std::cout << "  push ON  @t=" << t << " s\n" << std::flush;
    }
    if (pushApplied && !pushCleared && t >= t_push + dur) {
      robotInterface.clearExternalWrenches();
      pushCleared = true;
      std::cout << "  push OFF @t=" << t << " s\n" << std::flush;
    }

    const vector3_t p = st.getRootPositionInWorldFrame();
    last_z = p.z();
    min_z = std::min(min_z, last_z);
    if (baselineSet) {
      const double dx = p.x() - base_x0, dy = p.y() - base_y0;
      peak_disp = std::max(peak_disp, std::sqrt(dx * dx + dy * dy));
    }
    if (now >= nextLog) {
      std::cout << "  t=" << t << " s  base=(" << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;
      nextLog = now + std::chrono::milliseconds(500);
    }
    if (last_z < 0.3) {
      fell = true;
      std::cerr << "  fell at t=" << t << " s (base z=" << last_z << " m)" << std::endl;
      break;
    }
    std::this_thread::sleep_until(targetTime);
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - loopStart).count();

  // 8. Report. Recovered (in-regime) = did not fall AND peak base displacement stayed
  //    below the section-07 in-regime bound (0.12 m on the CoM; base used as proxy here).
  std::cout << "--- result ---\n";
  std::cout << "  control steps : " << iters << " over " << wall << " s wall  -> " << (iters / wall) << " Hz\n";
  std::cout << "  peak base horiz displacement : " << peak_disp << " m\n";
  std::cout << "  final base z  : " << last_z << " m   min base z = " << min_z << " m\n";
  const bool recovered = !fell && min_z > 0.5 && peak_disp < 0.12;
  std::cout << (recovered ? "[pushRecovery] PASS: MPC rejected the push in place\n"
              : fell      ? "[pushRecovery] FAIL: robot fell\n"
                          : "[pushRecovery] NOTE: stayed up but left the in-regime bound (peak_disp>=0.12 m)\n");

  std::cout.flush();
  std::cerr.flush();
  std::_Exit(recovered ? 0 : 2);  // sidestep the MRT controller's non-terminating solver thread (see standingClosedLoop)
}
