// Phase 0a deliverable (standalone, non-ROS 2): load the G1 + floor scene in
// MuJoCo and hold the default centroidal-MPC standing posture with per-joint PD.
// Foundation for the active-disturbance-rejection work (see
// DISTURBANCE_REJECTION_PLAN.md). Modeled on the sibling mujocoSimNoTorques.cpp
// (which only zeros torques and lets the robot fall), but adds a PD hold so the
// robot actually stands. Unlike the ROS 2 demo, paths are passed as argv instead
// of resolved via ament_index.
//
// Usage:
//   mujocoSimPDStand <scene.xml> <robot.urdf>
//   e.g. mujocoSimPDStand robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml \
//                         robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf

#include <mujoco_sim_interface/MujocoSimInterface.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

namespace {

// Default standing posture from
// robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference.info
// (defaultJointState block). Wrist joints (excluded from the MPC) are left at 0.
const std::unordered_map<std::string, double>& defaultJointPosture() {
  static const std::unordered_map<std::string, double> q = {
      {"left_hip_pitch_joint", -0.05},    {"left_hip_roll_joint", 0.0},        {"left_hip_yaw_joint", 0.0},
      {"left_knee_joint", 0.1},           {"left_ankle_pitch_joint", -0.05},   {"left_ankle_roll_joint", 0.0},
      {"right_hip_pitch_joint", -0.05},   {"right_hip_roll_joint", 0.0},       {"right_hip_yaw_joint", 0.0},
      {"right_knee_joint", 0.1},          {"right_ankle_pitch_joint", -0.05},  {"right_ankle_roll_joint", 0.0},
      {"waist_yaw_joint", 0.0},           {"waist_roll_joint", 0.0},           {"waist_pitch_joint", 0.0},
      {"left_shoulder_pitch_joint", 0.0}, {"left_shoulder_roll_joint", 0.0},   {"left_shoulder_yaw_joint", 0.0},
      {"left_elbow_joint", 0.0},          {"right_shoulder_pitch_joint", 0.0}, {"right_shoulder_roll_joint", 0.0},
      {"right_shoulder_yaw_joint", 0.0},  {"right_elbow_joint", 0.0},          {"left_wrist_roll_joint", 0.0},
      {"left_wrist_pitch_joint", 0.0},    {"left_wrist_yaw_joint", 0.0},       {"right_wrist_roll_joint", 0.0},
      {"right_wrist_pitch_joint", 0.0},   {"right_wrist_yaw_joint", 0.0},
  };
  return q;
}

constexpr double kDefaultBaseHeight = 0.7925;  // from reference.info
constexpr double kJointKp = 1500.0;
constexpr double kJointKd = 2.0;

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <scene.xml> <robot.urdf>\n"
              << "Example:\n  " << argv[0]
              << " robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml"
              << " robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf\n";
    return 1;
  }
  const std::string scenePath = argv[1];
  const std::string urdfPath = argv[2];
  // Optional 3rd arg "--headless": windowless self-check (no viewer) that steps
  // the sim a few seconds and verifies the robot stayed upright.
  const bool headless = (argc > 3 && std::string(argv[3]) == "--headless");
  if (!std::filesystem::exists(scenePath)) {
    std::cerr << "Scene file not found: " << scenePath << std::endl;
    return 1;
  }
  if (!std::filesystem::exists(urdfPath)) {
    std::cerr << "URDF file not found: " << urdfPath << std::endl;
    return 1;
  }

  // Build the desired initial state from the centroidal-MPC reference posture.
  robot::model::RobotDescription robotDescription(urdfPath);
  auto initStatePtr = std::make_shared<robot::model::RobotState>(robotDescription);
  initStatePtr->setConfigurationToZero();
  initStatePtr->setRootPositionInWorldFrame(robot::vector3_t(0.0, 0.0, kDefaultBaseHeight));
  for (const auto& [name, q_des] : defaultJointPosture()) {
    if (robotDescription.containsJoint(name)) {
      initStatePtr->setJointPosition(robotDescription.getJointIndex(name), q_des);
    }
  }

  // Configure and construct the sim.
  robot::mujoco_sim_interface::MujocoSimConfig config;
  config.scenePath = scenePath;
  config.dt = 0.001;
  config.initStatePtr_ = initStatePtr;
  config.verbose = true;
  config.headless = headless;

  robot::mujoco_sim_interface::MujocoSimInterface sim(config, urdfPath);

  // Stamp PD setpoints + gains into every joint action once; the sim thread
  // reads the thread-safe action each step.
  auto& action = sim.getRobotJointAction();
  for (const auto& [name, q_des] : defaultJointPosture()) {
    if (!robotDescription.containsJoint(name)) continue;
    auto& opt = action.at(robotDescription.getJointIndex(name));
    if (opt) {
      opt->q_des = q_des;
      opt->qd_des = 0.0;
      opt->kp = kJointKp;
      opt->kd = kJointKd;
      opt->feed_forward_effort = 0.0;
    }
  }
  sim.applyJointAction();

  // Start the sim (spawns the sim thread; plus the viewer unless headless).
  sim.startSim();

  if (headless) {
    // Windowless self-check: run ~3 s and confirm the robot stayed upright.
    for (int i = 0; i < 150; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      sim.updateInterfaceStateFromRobot();
    }
    const double z = sim.getRobotState().getRootPositionInWorldFrame().z();
    std::cout << "[pdstand] headless check: base height z=" << z << " m (start "
              << kDefaultBaseHeight << " m)" << std::endl;
    if (z > 0.5) {
      std::cout << "[pdstand] PASS - G1 is standing." << std::endl;
      return 0;
    }
    std::cerr << "[pdstand] FAIL - G1 fell (z <= 0.5 m)." << std::endl;
    return 2;
  }

  // Interactive viewer.
#if !defined(__APPLE__)
  // Linux: startSim() spawned the render thread (non-blocking); keep main alive.
  while (true) {
    sim.updateInterfaceStateFromRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
#endif
  // macOS: startSim() already drove the viewer on this thread and returned when
  // the window was closed — fall through and exit.
  return 0;
}
