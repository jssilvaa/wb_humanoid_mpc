// B3 rung 1 -- footstep-policy plumbing probe (open loop, no push, no capture-point logic).
//
// Verifies the bottom rung of the B3 verification ladder: that injecting a stepping gait through
// the ReactiveStepper SolverSynchronizedModule actually makes the closed-loop G1 lift and replant a
// foot, return to double stance, and stay standing -- i.e. that the footstep policy machinery
// (GaitSchedule/insertModeSequenceTemplate + SwingTrajectoryPlanner + the MRT controller's swing
// tracking) does what it is supposed to, BEFORE any reactive trigger is wired in.
//
// Same standing closed-loop wiring as standingClosedLoop/pushRecovery (headless MujocoSimInterface +
// CentroidalMpcMrtJointController), minus the push/feedforward; plus a single SCRIPTED step. The
// step is one insert of a COMPOSITE swing-then-stand template {swing, STANCE}: the swing foot lifts
// for swingDur and LANDS (the STANCE phase), then the robot holds double stance. A single-mode
// {swing} template would instead leave the foot in perpetual swing (it never touches down) and
// topple the robot -- the step must lift AND land.
//
// A per-step CSV of base / CoM / both foot positions (measured from MuJoCo) makes the swing arc
// (foot z ~ swingHeight 0.08 m) and the contact return directly visible.
//
// argv: stepProbe [sim_sec=6] [t_step=2.0] [swingDur=0.5] [foot=R] [csvPath] [taskFile]
//   foot: R = right foot swings (gait {LF}), L = left foot swings (gait {RF}). (real wall-clock sec.)

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
  const double t_step = (argc > 2) ? std::stod(argv[2]) : 2.0;    // swing onset [s]
  const double swingDur = (argc > 3) ? std::stod(argv[3]) : 0.5;  // swing-phase duration [s] (then lands + holds stance)
  const std::string footArg = (argc > 4 && std::string(argv[4]).size()) ? argv[4] : "R";  // which foot swings
  const std::string csvPath = (argc > 5 && std::string(argv[5]).size()) ? argv[5] : "";
  const std::string taskFile = (argc > 6 && std::string(argv[6]).size()) ? argv[6] : G1_TASK_FILE;
  const std::string urdfFile = G1_URDF_FILE;
  const std::string referenceFile = G1_REFERENCE_FILE;
  const std::string gaitFile = G1_GAIT_FILE;
  const std::string sceneFile = G1_SCENE_FILE;

  // Right foot swings -> left foot is the stance/contact foot -> mode LF (see MotionPhaseDefinition).
  const bool rightSwings = (footArg != "L" && footArg != "l");
  const size_t swingMode = rightSwings ? ModeNumber::LF : ModeNumber::RF;
  const std::string swingBody = rightSwings ? "right_ankle_roll_link" : "left_ankle_roll_link";
  // Composite single step: swing for swingDur, then land and hold double stance. The long final
  // stance (100 s) means the tiled template never steps a second time within any MPC horizon.
  const ModeSequenceTemplate stepGait({0.0, swingDur, swingDur + 100.0}, {swingMode, ModeNumber::STANCE});

  std::cout << "=== G1 centroidal-MPC step probe (headless, rung 1) ===\n";
  std::cout << "scripted step: " << (rightSwings ? "RIGHT" : "LEFT") << " foot swings (gait {" << modeNumber2String(swingMode)
            << ", STANCE}), onset t=" << t_step << " s, swingDur " << swingDur << " s, sim_T=" << sim_T << " s\n";

  // 1. Interface + SQP MPC.
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 2. Stance target (zero commanded velocity). ProceduralMpcMotionManager only supplies the
  //    TargetTrajectories here -- at zero velocity it leaves the gait schedule untouched (its
  //    currentGaitCommand_/lastGaitCommand_ both start "stance"), so ReactiveStepper owns the gait.
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});

  // Reactive stepper owns the gait schedule (prompt insertion, bypassing the 0.7*horizon heuristic).
  auto stepper = std::make_shared<ReactiveStepper>(interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule());

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

  // 7. Control loop, wall-clock bounded. Stage the swing gait at t_step, stance again at t_step+hold.
  double min_z = 1e9, last_z = 0.0;
  double swingZ0 = 0.0, peakSwingLift = 0.0, finalSwingLift = 0.0;
  bool swingZ0Set = false, stepStaged = false, fell = false;
  long iters = 0;

  std::ofstream csv;
  if (!csvPath.empty()) {
    csv.open(csvPath);
    csv.precision(9);
    csv << "t,base_x,base_y,base_z,com_x,com_y,com_z,footL_x,footL_y,footL_z,footR_x,footR_y,footR_z,stepping\n";
    std::cout << "  logging step CSV -> " << csvPath << "\n";
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

    // Scripted single step (idempotent edge): one insert of the swing-then-stand template.
    if (!stepStaged && t >= t_step) {
      stepper->setDesiredGait(stepGait);
      stepStaged = true;
      std::cout << "  step staged @t=" << t << " s\n" << std::flush;
    }

    const vector3_t base = st.getRootPositionInWorldFrame();
    const vector3_t footL = robotInterface.getBodyComPosition("left_ankle_roll_link");
    const vector3_t footR = robotInterface.getBodyComPosition("right_ankle_roll_link");
    const double swingZ = rightSwings ? footR.z() : footL.z();
    vector3_t com, vcom, angmom;
    robotInterface.getSubtreeCentroidalState(com, vcom, angmom);

    last_z = base.z();
    min_z = std::min(min_z, last_z);
    if (!swingZ0Set && t >= t_step - 0.05) {  // swing-foot rest height, just before the step
      swingZ0 = swingZ;
      swingZ0Set = true;
    }
    if (swingZ0Set) {
      peakSwingLift = std::max(peakSwingLift, swingZ - swingZ0);
      finalSwingLift = swingZ - swingZ0;
    }
    if (csv.is_open()) {
      csv << t << ',' << base.x() << ',' << base.y() << ',' << base.z() << ',' << com.x() << ',' << com.y() << ',' << com.z() << ','
          << footL.x() << ',' << footL.y() << ',' << footL.z() << ',' << footR.x() << ',' << footR.y() << ',' << footR.z() << ','
          << (stepStaged ? 1 : 0) << '\n';
    }
    if (now >= nextLog) {
      std::cout << "  t=" << t << " s  base_z=" << base.z() << "  swing_lift=" << (swingZ0Set ? swingZ - swingZ0 : 0.0) << std::endl;
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

  // 8. Report. A clean step = stayed up, the swing foot lifted clearly, and it came back down.
  std::cout << "--- result ---\n";
  std::cout << "  control steps   : " << iters << " over " << wall << " s wall  -> " << (iters / wall) << " Hz\n";
  std::cout << "  swing foot      : " << swingBody << "\n";
  std::cout << "  peak swing lift : " << peakSwingLift << " m (swing-height target 0.08 m)\n";
  std::cout << "  final swing lift: " << finalSwingLift << " m (should return ~0 after replant)\n";
  std::cout << "  final base z    : " << last_z << " m   min base z = " << min_z << " m\n";
  const bool lifted = peakSwingLift > 0.04;          // a real swing (vs noise) -- target is 0.08 m
  const bool replanted = std::abs(finalSwingLift) < 0.02;  // foot back down at the end
  const bool stoodUp = !fell && min_z > 0.5;
  const bool clean = stoodUp && lifted && replanted;
  std::cout << (clean ? "[stepProbe] PASS: single step lifted and replanted, robot stayed up\n"
               : fell ? "[stepProbe] FAIL: robot fell\n"
                      : "[stepProbe] NOTE: stayed up but step looks off (see peak/final swing lift)\n");

  if (csv.is_open()) csv.close();  // flush before _Exit (skips destructors / stream flushing)
  std::cout.flush();
  std::cerr.flush();
  std::_Exit(clean ? 0 : 2);  // sidestep the MRT controller's non-terminating solver thread
}
