// B3 closed-loop probe -- reactive stepping verification ladder (rungs 1-3).
//
// Standing closed loop (headless MujocoSimInterface + CentroidalMpcMrtJointController) + the
// ReactiveStepper SolverSynchronizedModule + an optional scripted xfrc push. Logs, every control
// step, the capture point xi = c + cdot/omega computed two ways:
//   - module: from the MPC init state via Pinocchio (ReactiveStepper::getCaptureState) -- the exact
//     quantity the stepping trigger will test;
//   - MuJoCo: from the measured subtree momentum (getSubtreeCentroidalState) -- ground truth;
// plus the module's foot positions (the support polygon). This validates the CP computation and
// characterises where the CP sits relative to support under a push (rung 2).
//
// stepTrigger:
//   off       : never step (rung 2 -- push + CP logging only)
//   scripted  : stage one composite {swing, STANCE} step at t_push (rung 1 plumbing check). A
//               single-mode {swing} template would leave the foot in perpetual swing and topple;
//               the step must lift AND land.
//   auto      : capture-point FSM (rung 3) -- not yet implemented; falls back to off.
//
// argv: stepProbe [sim_s=6] [fx=0] [fy=0] [t_push=2] [dur=0.1] [stepTrigger=off] [foot=R] [csv] [taskFile]
//   foot: R = right swings (gait {LF}), L = left swings (gait {RF}). (real wall-clock seconds.)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <ocs2_sqp/SqpMpc.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h>
#include <humanoid_centroidal_mpc/synchronized_module/ReactiveStepper.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include <mujoco/mujoco.h>
#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <robot_model/RobotDescription.h>
#include <robot_model/RobotState.h>

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
  const double sim_T = (argc > 1) ? std::stod(argv[1]) : 6.0;     // real (wall-clock) seconds
  const double fx = (argc > 2) ? std::stod(argv[2]) : 0.0;        // push force x [N]
  const double fy = (argc > 3) ? std::stod(argv[3]) : 0.0;        // push force y [N]
  const double t_push = (argc > 4) ? std::stod(argv[4]) : 2.0;    // push (and scripted-step) onset [s]
  const double dur = (argc > 5) ? std::stod(argv[5]) : 0.1;       // push duration [s]
  const std::string stepTrigger = (argc > 6 && std::string(argv[6]).size()) ? argv[6] : "off";
  const std::string footArg = (argc > 7 && std::string(argv[7]).size()) ? argv[7] : "R";
  const std::string csvPath = (argc > 8 && std::string(argv[8]).size()) ? argv[8] : "";
  const std::string taskFile = (argc > 9 && std::string(argv[9]).size()) ? argv[9] : G1_TASK_FILE;
  const std::string urdfFile = G1_URDF_FILE;
  const std::string referenceFile = G1_REFERENCE_FILE;
  const std::string gaitFile = G1_GAIT_FILE;
  const std::string sceneFile = G1_SCENE_FILE;
  const double swingDur = 0.5;  // swing-phase duration of the scripted step [s]

  const bool scripted = (stepTrigger == "scripted");
  const bool autoTrig = (stepTrigger == "auto");
  // Right foot swings -> left foot is the stance/contact foot -> mode LF (see MotionPhaseDefinition).
  const bool rightSwings = (footArg != "L" && footArg != "l");
  const size_t swingMode = rightSwings ? ModeNumber::LF : ModeNumber::RF;
  // Composite single step: swing for swingDur, then land and hold double stance (long final phase
  // so the tiled template never steps a second time within any horizon).
  const ModeSequenceTemplate stepGait({0.0, swingDur, swingDur + 100.0}, {swingMode, ModeNumber::STANCE});

  std::cout << "=== G1 centroidal-MPC step probe (headless) ===\n";
  std::cout << "push f=(" << fx << ", " << fy << ", 0) N at torso_link t=[" << t_push << ", " << (t_push + dur)
            << "] s; stepTrigger=" << stepTrigger << (scripted ? (rightSwings ? " (RIGHT swings)" : " (LEFT swings)") : "")
            << "; sim_T=" << sim_T << " s\n";

  // 1. Interface + SQP MPC.
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 2. Stance target (zero commanded velocity). ProceduralMpcMotionManager only supplies the
  //    TargetTrajectories here -- at zero velocity it leaves the gait schedule untouched, so the
  //    ReactiveStepper owns the gait.
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});

  auto stepper = std::make_shared<ReactiveStepper>(interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
                                                   interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                                                   interface.modelSettings());
  if (autoTrig) {
    stepper->enableAutoStepping();
    std::cout << "  auto capture-point stepping ENABLED\n";
  }

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(motionManager);
  mpc.getSolverPtr()->addSynchronizedModule(stepper);

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
      std::cerr << "[stepProbe] FAIL: no initial MPC policy within 60 s\n";
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  robotInterface.startSim();

  // 7. Control loop, wall-clock bounded.
  const vector3_t pushForce(fx, fy, 0.0);
  const bool hasPush = (std::abs(fx) + std::abs(fy) > 1e-9);
  const double g = std::abs(robotInterface.getModel()->opt.gravity[2]);
  double min_z = 1e9, last_z = 0.0;
  double cpErrSum = 0.0;
  long cpErrN = 0;
  bool stepStaged = false, pushApplied = false, pushCleared = false, fell = false;
  long iters = 0;

  std::ofstream csv;
  if (!csvPath.empty()) {
    csv.open(csvPath);
    csv.precision(9);
    csv << "t,pushing,base_z,"
           "mj_com_x,mj_com_y,mj_com_z,mj_cp_x,mj_cp_y,"        // MuJoCo ground truth
           "md_com_x,md_com_y,md_com_z,md_cp_x,md_cp_y,md_omega,md_vx,md_vy,md_rhacv_x,md_rhacv_y,"  // module + raw/smoothed CoM vel
           "md_footL_x,md_footL_y,md_footR_x,md_footR_y,"       // module support polygon
           "md_outside,md_fsm,md_steps\n";                       // FSM: CP outside support / state / steps
    std::cout << "  logging CP CSV -> " << csvPath << "\n";
  }

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

    // Scripted single step (idempotent edge).
    if (scripted && !stepStaged && t >= t_push) {
      stepper->setDesiredGait(stepGait);
      stepStaged = true;
      std::cout << "  step staged @t=" << t << " s\n" << std::flush;
    }
    // Push window.
    if (hasPush && !pushApplied && t >= t_push) {
      robotInterface.setExternalWrench("torso_link", pushForce);
      pushApplied = true;
      std::cout << "  push ON  @t=" << t << " s\n" << std::flush;
    }
    if (pushApplied && !pushCleared && t >= t_push + dur) {
      robotInterface.clearExternalWrenches();
      pushCleared = true;
      std::cout << "  push OFF @t=" << t << " s\n" << std::flush;
    }

    // MuJoCo ground-truth CP.
    vector3_t com, vcom, angmom;
    robotInterface.getSubtreeCentroidalState(com, vcom, angmom);
    const double omega_mj = std::sqrt(g / std::max(com.z(), 1e-3));
    const double mj_cp_x = com.x() + vcom.x() / omega_mj;
    const double mj_cp_y = com.y() + vcom.y() / omega_mj;

    // Module CP (from the MPC init state, the trigger's quantity).
    const ReactiveStepper::CaptureState cs = stepper->getCaptureState();

    const vector3_t base = st.getRootPositionInWorldFrame();
    last_z = base.z();
    min_z = std::min(min_z, last_z);
    if (cs.valid) {  // accumulate module-vs-MuJoCo CP agreement
      cpErrSum += std::hypot(cs.capturePoint.x() - mj_cp_x, cs.capturePoint.y() - mj_cp_y);
      ++cpErrN;
    }
    if (csv.is_open()) {
      csv << t << ',' << (pushApplied && !pushCleared ? 1 : 0) << ',' << base.z() << ',' << com.x() << ',' << com.y() << ','
          << com.z() << ',' << mj_cp_x << ',' << mj_cp_y << ',' << cs.com.x() << ',' << cs.com.y() << ',' << cs.com.z() << ','
          << cs.capturePoint.x() << ',' << cs.capturePoint.y() << ',' << cs.omega << ',' << cs.comVel.x() << ',' << cs.comVel.y()
          << ',' << cs.rhacv.x() << ',' << cs.rhacv.y() << ',' << cs.footL.x() << ',' << cs.footL.y()
          << ',' << cs.footR.x() << ',' << cs.footR.y() << ',' << (cs.outsideSupport ? 1 : 0) << ',' << cs.fsmState << ','
          << cs.stepCount << '\n';
    }
    if (now >= nextLog) {
      std::cout << "  t=" << t << "  base_z=" << base.z() << "  cp_mj=(" << mj_cp_x << ", " << mj_cp_y << ")  cp_mod=("
                << cs.capturePoint.x() << ", " << cs.capturePoint.y() << ")" << std::endl;
      nextLog = now + std::chrono::milliseconds(250);
    }
    if (last_z < 0.3) {
      fell = true;
      std::cerr << "  fell at t=" << t << " s (base z=" << last_z << " m)" << std::endl;
      break;
    }
    std::this_thread::sleep_until(targetTime);
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - loopStart).count();

  // 8. Report.
  std::cout << "--- result ---\n";
  std::cout << "  control steps : " << iters << " over " << wall << " s wall  -> " << (iters / wall) << " Hz\n";
  std::cout << "  final base z   : " << last_z << " m   min base z = " << min_z << " m\n";
  std::cout << "  steps staged   : " << stepper->getCaptureState().stepCount << "\n";
  std::cout << "  mean |CP_module - CP_mujoco| : " << (cpErrN ? cpErrSum / cpErrN : 0.0) << " m  (" << cpErrN << " samples)\n";
  const bool stoodUp = !fell && min_z > 0.5;
  std::cout << (stoodUp ? "[stepProbe] PASS: robot stayed up\n" : fell ? "[stepProbe] FAIL: robot fell\n" : "[stepProbe] NOTE: left the standing regime\n");
  // Machine-readable summary for the seed campaign (fall = min base z < 0.5, per report section 07).
  std::cout << "B3SUMMARY min_z=" << min_z << " steps=" << stepper->getCaptureState().stepCount << " fell=" << (min_z < 0.5 ? 1 : 0)
            << "\n";

  if (csv.is_open()) csv.close();  // flush before _Exit (skips destructors / stream flushing)
  std::cout.flush();
  std::cerr.flush();
  std::_Exit(stoodUp ? 0 : 2);  // sidestep the MRT controller's non-terminating solver thread
}
