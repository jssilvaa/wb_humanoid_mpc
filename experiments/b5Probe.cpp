// B5 closed-loop probe -- integrate observer-driven wrench feedforward (B2) WITH reactive
// CP+RHACV stepping (B3) in ONE harness, to test whether the two ADR mechanisms cooperate
// (H1), whether the feedforward delays the step (H2, anti-phase), or whether keeping the
// feedforward on through single support saturates the stance CWC and destabilises the swing
// (H3). One control loop and one fall criterion (min base z < 0.5 m, per report section 07),
// so the arms are directly comparable.
//
// mode (argv[6]):
//   off  : no ADR (baseline)          -- stepper passive (CP logged, never triggers), no FF.
//   ff   : observer feedforward only  -- the B2 arm (stepper passive).
//   step : reactive stepping only     -- the B3 arm (no feedforward).
//   both : feedforward + stepping     -- the B5 integration; FF kept ON through swing.
//
// In ff/both the cascaded centroidal-momentum observer (B2.1) estimates W_hat from the
// measured centroidal momentum + ground reaction every control step and feeds it to the
// ExternalWrenchFeedforward module (same wiring as pushRecovery ff_mode=2; the ground
// reaction is read live, so single support during a step is handled). The ReactiveStepper is
// always added as a synchronized module; enableAutoStepping() is called only in step/both, so
// off/ff log the capture point passively without touching the gait.
//
// argv: b5Probe [sim_s=6] [fx=0] [fy=0] [t_push=2] [dur=0.1] [mode=both] [foot=R] [csv] [taskFile] [ff_T=0.27]
//   foot: R = right swings (gait {LF}), L = left swings (gait {RF}). (real wall-clock seconds.)
//
// Prints "B5SUMMARY min_z= steps= first_step_t= what_peak= fell=" for the seed/sweep script.

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
#include <humanoid_centroidal_mpc/synchronized_module/ExternalWrenchFeedforward.h>
#include <humanoid_centroidal_mpc/synchronized_module/ReactiveStepper.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include <humanoid_disturbance_rejection/CentroidalMomentumObserver.h>
#include <mujoco/mujoco.h>
#include <mujoco_sim_interface/MujocoSimInterface.h>
#include <robot_model/RobotDescription.h>
#include <robot_model/RobotState.h>

using namespace ocs2;
using namespace ocs2::humanoid;
namespace adr = ::humanoid::adr;  // cascaded centroidal-momentum observer (B2.1)

int main(int argc, char** argv) {
  const double sim_T = (argc > 1) ? std::stod(argv[1]) : 6.0;     // real (wall-clock) seconds
  const double fx = (argc > 2) ? std::stod(argv[2]) : 0.0;        // push force x [N]
  const double fy = (argc > 3) ? std::stod(argv[3]) : 0.0;        // push force y [N]
  const double t_push = (argc > 4) ? std::stod(argv[4]) : 2.0;    // push onset [s]
  const double dur = (argc > 5) ? std::stod(argv[5]) : 0.1;       // push duration [s]
  const std::string mode = (argc > 6 && std::string(argv[6]).size()) ? argv[6] : "both";
  const std::string footArg = (argc > 7 && std::string(argv[7]).size()) ? argv[7] : "R";
  const std::string csvPath = (argc > 8 && std::string(argv[8]).size()) ? argv[8] : "";
  const std::string taskFile = (argc > 9 && std::string(argv[9]).size()) ? argv[9] : G1_TASK_FILE;
  const double ff_T = (argc > 10) ? std::stod(argv[10]) : 0.27;   // horizon decay T [s] (B2 knee ~ sqrt(z/g))
  const std::string urdfFile = G1_URDF_FILE;
  const std::string referenceFile = G1_REFERENCE_FILE;
  const std::string gaitFile = G1_GAIT_FILE;
  const std::string sceneFile = G1_SCENE_FILE;

  if (mode != "off" && mode != "ff" && mode != "step" && mode != "both") {
    std::cerr << "[b5Probe] FAIL: mode must be one of {off, ff, step, both}, got '" << mode << "'\n";
    return 2;
  }
  const bool stepOn = (mode == "step" || mode == "both");  // reactive stepping (B3)
  const bool ffOn = (mode == "ff" || mode == "both");      // observer feedforward (B2)
  const bool rightSwings = (footArg != "L" && footArg != "l");

  std::cout << "=== G1 centroidal-MPC B5 probe (headless) ===\n";
  std::cout << "push f=(" << fx << ", " << fy << ", 0) N at torso_link t=[" << t_push << ", " << (t_push + dur)
            << "] s; mode=" << mode << " (stepping=" << (stepOn ? "on" : "off") << ", feedforward=" << (ffOn ? "on" : "off")
            << "); ff_T=" << ff_T << " s; sim_T=" << sim_T << " s\n";

  // 1. Interface + SQP MPC.
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // 2. Stance target (zero commanded velocity). At zero velocity ProceduralMpcMotionManager leaves
  //    the gait schedule untouched, so the ReactiveStepper owns the gait.
  CentroidalMpcTargetTrajectoriesCalculator targetCalc(referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
                                                       interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetFunc =
      [&targetCalc](const vector4_t& vel, scalar_t t0, scalar_t /*tf*/, const vector_t& x0) mutable {
        return targetCalc.commandedVelocityToTargetTrajectories(vel, t0, x0);
      };
  auto motionManager = std::make_shared<ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetFunc);
  motionManager->setAndScaleVelocityCommand(WalkingVelocityCommand{});

  // Reactive stepper (B3) -- always added; auto-stepping enabled only in step/both, so off/ff log
  // the capture point passively without changing the gait.
  auto stepper = std::make_shared<ReactiveStepper>(interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
                                                   interface.getPinocchioInterface(), interface.getMpcRobotModel(),
                                                   interface.modelSettings());
  if (stepOn) {
    stepper->enableAutoStepping();
    std::cout << "  reactive capture-point stepping ENABLED\n";
  }

  // External-wrench feedforward (B2) -- observer-driven. The module freezes the latest estimate into
  // the dynamics buffer once per solve; the cloned dynamics add decay*W_hat/mass to the momentum rate.
  std::shared_ptr<ExternalWrenchFeedforward> ffModule;
  if (ffOn) {
    ffModule = std::make_shared<ExternalWrenchFeedforward>(interface.getExternalWrenchFeedforwardPtr(), ff_T, /*trust=*/1.0);
    std::cout << "  observer-driven external-wrench feedforward ENABLED (horizon decay T=" << ff_T << " s)\n";
  }

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(motionManager);
  mpc.getSolverPtr()->addSynchronizedModule(stepper);
  if (ffModule) mpc.getSolverPtr()->addSynchronizedModule(ffModule);

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
      std::cerr << "[b5Probe] FAIL: no initial MPC policy within 60 s\n";
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  robotInterface.startSim();

  // Observer (ff/both): cascaded centroidal-momentum observer, same config as pushRecovery (B2.1).
  adr::CentroidalObserverConfig obsCfg;
  obsCfg.order = 3;
  obsCfg.dt = 0.002;  // nominal fallback; overridden per step with the measured dt
  obsCfg.profile = adr::ObserverGainProfile::ShortPulse;  // alpha = 70 (centroidal default)
  adr::CentroidalMomentumObserver observer(obsCfg);
  const double g = std::abs(robotInterface.getModel()->opt.gravity[2]);
  const double totalMass = mj_getTotalmass(robotInterface.getModel());
  const vector3_t gravityVec(0.0, 0.0, -g);
  double obsPrevT = -1.0;

  // 7. Control loop, wall-clock bounded.
  const vector3_t pushForce(fx, fy, 0.0);
  const bool hasPush = (std::abs(fx) + std::abs(fy) > 1e-9);
  double min_z = 1e9, last_z = 0.0;
  double whatPeak = 0.0, firstStepT = -1.0;
  bool pushApplied = false, pushCleared = false, fell = false;
  long iters = 0;

  std::ofstream csv;
  if (!csvPath.empty()) {
    csv.open(csvPath);
    csv.precision(9);
    csv << "t,pushing,base_z,"
           "mj_com_x,mj_com_y,mj_cp_x,mj_cp_y,"                 // MuJoCo ground truth
           "md_cp_x,md_cp_y,md_rhacv_x,md_rhacv_y,"            // module CP + smoothed CoM vel
           "md_outside,md_fsm,md_steps,"                        // FSM: CP outside support / state / steps
           "ff_on,what_x,what_y,what_tz,what_norm\n";           // observer feedforward W_hat = [f; tau]
    std::cout << "  logging CSV -> " << csvPath << "\n";
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

    // Push window (idempotent edges).
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

    // MuJoCo ground-truth centroidal state + CP.
    vector3_t com, vcom, angmom;
    robotInterface.getSubtreeCentroidalState(com, vcom, angmom);
    const double omega_mj = std::sqrt(g / std::max(com.z(), 1e-3));
    const double mj_cp_x = com.x() + vcom.x() / omega_mj;
    const double mj_cp_y = com.y() + vcom.y() / omega_mj;

    // Observer feedforward source (ff/both): estimate W_hat from measured momentum + ground reaction
    // every step, then hand it to the feedforward module. The ground reaction is read live, so single
    // support during a step (swing foot ~ 0) is handled without special-casing.
    vector6_t wHat = vector6_t::Zero();
    if (ffModule) {
      adr::vector6_t hMeas;
      hMeas.head<3>() = totalMass * vcom;
      hMeas.tail<3>() = angmom;
      adr::vector6_t wKnown;
      wKnown.head<3>() = totalMass * gravityVec;
      wKnown.tail<3>().setZero();
      wKnown += robotInterface.getGroundReactionWrench(com);  // + ground reaction [f; tau] about CoM
      // Clamp the observer dt to the forward-Euler stability region. The control loop is variable-rate
      // (sleep_until-paced); an occasional scheduling hiccup makes the raw wall-clock dt = t - obsPrevT
      // exceed ~2/max_gain = 2/210 ~ 9.5 ms, which destabilises the order-3 cascade and blows W_hat up
      // (the QP then throws "MPC has crashed!", or a garbage feedforward topples the robot). Capping at
      // 4 ms keeps the integrator well inside its stable region without distorting on-rate steps (the
      // nominal control period is 2 ms). This is a harness-level guard; the observer is left identical
      // to the B2 build for comparability.
      const double rawDt = (obsPrevT >= 0.0) ? (t - obsPrevT) : 0.002;
      obsPrevT = t;
      const double obsDt = std::clamp(rawDt, 1e-4, 4e-3);
      wHat = observer.update(hMeas, wKnown, obsDt);
      if (!wHat.allFinite()) wHat.setZero();  // safety net (should not trigger once dt is bounded)
      vector_t wFF = wHat;  // fixed (6) -> dynamic vector_t for the feedforward buffer
      ffModule->setWrench(wFF);
      whatPeak = std::max(whatPeak, wHat.head<3>().norm());
    }

    // Module capture state (from the MPC init state -- the trigger's quantity).
    const ReactiveStepper::CaptureState cs = stepper->getCaptureState();
    if (firstStepT < 0.0 && cs.stepCount > 0) firstStepT = t;  // first step onset (FF-delay diagnostic)

    const vector3_t base = st.getRootPositionInWorldFrame();
    last_z = base.z();
    min_z = std::min(min_z, last_z);
    if (csv.is_open()) {
      csv << t << ',' << (pushApplied && !pushCleared ? 1 : 0) << ',' << base.z() << ',' << com.x() << ',' << com.y() << ','
          << mj_cp_x << ',' << mj_cp_y << ',' << cs.capturePoint.x() << ',' << cs.capturePoint.y() << ',' << cs.rhacv.x() << ','
          << cs.rhacv.y() << ',' << (cs.outsideSupport ? 1 : 0) << ',' << cs.fsmState << ',' << cs.stepCount << ','
          << (ffModule ? 1 : 0) << ',' << wHat(0) << ',' << wHat(1) << ',' << wHat(5) << ',' << wHat.head<3>().norm() << '\n';
    }
    if (now >= nextLog) {
      std::cout << "  t=" << t << "  base_z=" << base.z() << "  cp_mod=(" << cs.capturePoint.x() << ", " << cs.capturePoint.y()
                << ")  fsm=" << cs.fsmState << "  steps=" << cs.stepCount << "  |f_hat|=" << wHat.head<3>().norm() << std::endl;
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
  const long steps = stepper->getCaptureState().stepCount;
  std::cout << "--- result ---\n";
  std::cout << "  control steps  : " << iters << " over " << wall << " s wall  -> " << (iters / wall) << " Hz\n";
  std::cout << "  final base z    : " << last_z << " m   min base z = " << min_z << " m\n";
  std::cout << "  steps taken     : " << steps << (firstStepT >= 0.0 ? "  (first @t=" + std::to_string(firstStepT) + " s)" : "") << "\n";
  std::cout << "  peak |f_hat|    : " << whatPeak << " N\n";
  const bool stoodUp = !fell && min_z > 0.5;
  std::cout << (stoodUp ? "[b5Probe] PASS: robot stayed up\n" : fell ? "[b5Probe] FAIL: robot fell\n" : "[b5Probe] NOTE: left the standing regime\n");
  // Machine-readable summary for the sweep script (fall = min base z < 0.5, per report section 07).
  std::cout << "B5SUMMARY min_z=" << min_z << " steps=" << steps << " first_step_t=" << firstStepT << " what_peak=" << whatPeak
            << " fell=" << (min_z < 0.5 ? 1 : 0) << "\n";

  if (csv.is_open()) csv.close();  // flush before _Exit (skips destructors / stream flushing)
  std::cout.flush();
  std::cerr.flush();
  std::_Exit(stoodUp ? 0 : 2);  // sidestep the MRT controller's non-terminating solver thread
}
