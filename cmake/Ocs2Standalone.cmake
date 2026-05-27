# =============================================================================
# Standalone (non-ROS 2) build of the OCS2 packages needed for the centroidal MPC.
# Included from the root CMakeLists when -DBUILD_OCS2=ON. Each package is a plain
# static lib (no ament), built bottom-up from wb's FULL lib/ocs2_ros2 sources.
# See DISTURBANCE_REJECTION_PLAN.md (Phase 0b) for the full ladder + gotchas.
# Reuses vic-mpc's substrate find-recipes; built incrementally (one package at a time).
# =============================================================================

set(OCS2 ${CMAKE_CURRENT_SOURCE_DIR}/lib/ocs2_ros2)

# ---- shared deps -----------------------------------------------------------
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED NO_MODULE)
endif()
find_package(Threads REQUIRED)
find_package(OpenMP)  # optional; OCS2 uses it for parallelism

# Boost: brew 1.90's BoostConfig component lookup is unreliable, so locate the
# few compiled libs directly (headers come via /opt/homebrew/include below).
find_package(Boost REQUIRED)  # headers / Boost_INCLUDE_DIRS
set(OCS2_BOOST_LIBS "")
foreach(comp system filesystem log log_setup)
  find_library(BOOST_${comp}_LIB NAMES boost_${comp}
    HINTS /opt/homebrew/lib /usr/local/lib ${Boost_LIBRARY_DIRS})
  if(BOOST_${comp}_LIB)
    list(APPEND OCS2_BOOST_LIBS ${BOOST_${comp}_LIB})
  endif()
endforeach()

# Common OCS2 compile settings (macOS/clang/C++20). Deliberately NOT using
# ocs2_core/cmake/ocs2_cxx_flags.cmake (forces C++14 + GNU-ld -Wl,--no-as-needed).
add_library(ocs2_flags INTERFACE)
# C++20 (unified with the robot_runtime/MuJoCo/observer side). OCS2 originally used
# std::result_of (removed in C++20 on libc++); patched to std::invoke_result in 3
# ocs2_core headers (see OCS2_CLANG_PATCHES.md). Unifying at C++20 is REQUIRED for the
# closed-loop harness: CentroidalMpcMrtJointController includes BOTH OCS2 headers and
# robot_model headers (which use C++20 concepts/span via IDMapBase.h) in one TU.
target_compile_options(ocs2_flags INTERFACE
  -std=gnu++20 -Wno-invalid-partial-specialization
  -include cassert   # OCS2 headers use assert() but rely on a transitive <cassert>
  -O2)               # -O2: OCS2's numerical inner loops are unusably slow at -O0 (LQ approx ~6ms -> sub-ms)
target_compile_definitions(ocs2_flags INTERFACE BOOST_MPL_LIMIT_LIST_SIZE=30)
if(APPLE AND EXISTS /opt/homebrew/include)
  target_include_directories(ocs2_flags SYSTEM INTERFACE /opt/homebrew/include)  # Boost headers
endif()

# ---- 1) ocs2_thirdparty : header-only (CppAD + CppADCodeGen + iit) ----------
add_library(ocs2_third_party INTERFACE)
target_include_directories(ocs2_third_party INTERFACE ${OCS2}/ocs2_thirdparty/include)
target_link_libraries(ocs2_third_party INTERFACE ${CMAKE_DL_LIBS})
add_library(ocs2::third_party ALIAS ocs2_third_party)

# ---- 2) ocs2_core : foundational risk gate (full source) -------------------
file(GLOB_RECURSE OCS2_CORE_SRC CONFIGURE_DEPENDS ${OCS2}/ocs2_core/src/*.cpp)
add_library(ocs2_core STATIC ${OCS2_CORE_SRC})
target_include_directories(ocs2_core PUBLIC ${OCS2}/ocs2_core/include)
target_link_libraries(ocs2_core PUBLIC
  ocs2::third_party ocs2_flags Eigen3::Eigen Threads::Threads
  ${CMAKE_DL_LIBS} ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_core PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::core ALIAS ocs2_core)

# ---- 3) ocs2_oc : full (risk gate 2) ---------------------------------------
file(GLOB_RECURSE OCS2_OC_SRC CONFIGURE_DEPENDS ${OCS2}/ocs2_oc/src/*.cpp)
add_library(ocs2_oc STATIC ${OCS2_OC_SRC})
target_include_directories(ocs2_oc PUBLIC ${OCS2}/ocs2_oc/include)
target_link_libraries(ocs2_oc PUBLIC ocs2::core ocs2_flags Eigen3::Eigen ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_oc PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::oc ALIAS ocs2_oc)

# ---- 4) ocs2_robotic_tools -------------------------------------------------
file(GLOB_RECURSE OCS2_RT_SRC CONFIGURE_DEPENDS ${OCS2}/ocs2_robotic_tools/src/*.cpp)
add_library(ocs2_robotic_tools STATIC ${OCS2_RT_SRC})
target_include_directories(ocs2_robotic_tools PUBLIC ${OCS2}/ocs2_robotic_tools/include)
target_link_libraries(ocs2_robotic_tools PUBLIC ocs2::oc ocs2::core ocs2_flags Eigen3::Eigen ${OCS2_BOOST_LIBS})
add_library(ocs2::robotic_tools ALIAS ocs2_robotic_tools)

# ---- 5) ocs2_pinocchio_interface (brew pinocchio/urdfdom; reuses root finds) -
set(OCS2_PININT ${OCS2}/ocs2_pinocchio/ocs2_pinocchio_interface)
file(GLOB_RECURSE OCS2_PININT_SRC CONFIGURE_DEPENDS ${OCS2_PININT}/src/*.cpp)
add_library(ocs2_pinocchio_interface STATIC ${OCS2_PININT_SRC})
target_include_directories(ocs2_pinocchio_interface PUBLIC
  ${OCS2_PININT}/include
  ${URDFDOM_PREFIX_INCLUDE} ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE} ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)
target_link_libraries(ocs2_pinocchio_interface PUBLIC
  ocs2::core ocs2::robotic_tools ocs2_flags PkgConfig::pinocchio Eigen3::Eigen
  ${urdfdom_LIBRARIES} ${OCS2_BOOST_LIBS})
add_library(ocs2::pinocchio_interface ALIAS ocs2_pinocchio_interface)

# ---- 6) ocs2_centroidal_model ----------------------------------------------
set(OCS2_CENTM ${OCS2}/ocs2_pinocchio/ocs2_centroidal_model)
file(GLOB_RECURSE OCS2_CENTM_SRC CONFIGURE_DEPENDS ${OCS2_CENTM}/src/*.cpp)
add_library(ocs2_centroidal_model STATIC ${OCS2_CENTM_SRC})
target_include_directories(ocs2_centroidal_model PUBLIC ${OCS2_CENTM}/include)
target_link_libraries(ocs2_centroidal_model PUBLIC
  ocs2::core ocs2::pinocchio_interface ocs2::robotic_tools ocs2_flags
  PkgConfig::pinocchio Eigen3::Eigen ${OCS2_BOOST_LIBS})
add_library(ocs2::centroidal_model ALIAS ocs2_centroidal_model)

# ---- 7) ocs2_qp_solver (dense KKT; no HPIPM) -------------------------------
set(OCS2_QPS ${OCS2}/ocs2_test_tools/ocs2_qp_solver)
file(GLOB_RECURSE OCS2_QPS_SRC CONFIGURE_DEPENDS ${OCS2_QPS}/src/*.cpp)
add_library(ocs2_qp_solver STATIC ${OCS2_QPS_SRC})
target_include_directories(ocs2_qp_solver PUBLIC ${OCS2_QPS}/include)
target_link_libraries(ocs2_qp_solver PUBLIC ocs2::core ocs2::oc ocs2_flags Eigen3::Eigen ${OCS2_BOOST_LIBS})
add_library(ocs2::qp_solver ALIAS ocs2_qp_solver)

# ---- 8/9) BLASFEO + HPIPM --------------------------------------------------
# Built from source at OCS2's pinned tags by ./build_hpipm.sh (portable: Ubuntu +
# macOS), installed to external/install. NOTE: must match the tags wb's
# HpipmInterface.cpp targets — a different hpipm tag mismatches d_ocp_qp_dim_set_all.
set(HPIPM_INSTALL_DIR "${CMAKE_SOURCE_DIR}/external/install" CACHE PATH "blasfeo+hpipm prefix (run ./build_hpipm.sh)")
add_library(blasfeo STATIC IMPORTED GLOBAL)
set_target_properties(blasfeo PROPERTIES
  IMPORTED_LOCATION ${HPIPM_INSTALL_DIR}/lib/libblasfeo.a
  INTERFACE_INCLUDE_DIRECTORIES ${HPIPM_INSTALL_DIR}/include)
add_library(hpipm STATIC IMPORTED GLOBAL)
set_target_properties(hpipm PROPERTIES
  IMPORTED_LOCATION ${HPIPM_INSTALL_DIR}/lib/libhpipm.a
  INTERFACE_INCLUDE_DIRECTORIES ${HPIPM_INSTALL_DIR}/include)
target_link_libraries(hpipm INTERFACE blasfeo m)  # hpipm before blasfeo, + libm

# ---- 10) hpipm_catkin : OCS2 <-> HPIPM bridge ------------------------------
set(OCS2_HPIPMC ${OCS2}/ocs2_sqp/hpipm_catkin)
add_library(ocs2_hpipm_interface STATIC
  ${OCS2_HPIPMC}/src/HpipmInterface.cpp ${OCS2_HPIPMC}/src/HpipmInterfaceSettings.cpp)
target_include_directories(ocs2_hpipm_interface PUBLIC ${OCS2_HPIPMC}/include)
target_link_libraries(ocs2_hpipm_interface PUBLIC
  ocs2::core ocs2::oc ocs2_flags hpipm Eigen3::Eigen ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_hpipm_interface PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::hpipm_interface ALIAS ocs2_hpipm_interface)

# ---- 11) ocs2_mpc ----------------------------------------------------------
file(GLOB_RECURSE OCS2_MPC_SRC CONFIGURE_DEPENDS ${OCS2}/ocs2_mpc/src/*.cpp)
list(FILTER OCS2_MPC_SRC EXCLUDE REGEX "lintTarget")
add_library(ocs2_mpc STATIC ${OCS2_MPC_SRC})
target_include_directories(ocs2_mpc PUBLIC ${OCS2}/ocs2_mpc/include)
target_link_libraries(ocs2_mpc PUBLIC ocs2::core ocs2::oc ocs2_flags Eigen3::Eigen ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_mpc PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::mpc ALIAS ocs2_mpc)

# ---- 12) ocs2_ddp : SETTINGS ONLY ------------------------------------------
# This repo solves with SQP; the centroidal MPC needs ocs2_ddp purely for the
# ddp::Settings struct + ddp::loadSettings (it parses a "ddp" block from task.info).
# We deliberately do NOT build the DDP solver itself (GaussNewtonDDP/SLQ/ILQR/
# DDP_DataCollector): DDP_DataCollector references OCS2 API removed in this version
# (ConstraintBase/CostFunctionBase) — DDP is unmaintained in this fork. The two TUs
# below are settings loaders only (ocs2_core + boost); StrategySettings provides the
# search_strategy/line_search/levenberg_marquardt loaders that ddp::loadSettings calls.
add_library(ocs2_ddp STATIC
  ${OCS2}/ocs2_ddp/src/DDP_Settings.cpp
  ${OCS2}/ocs2_ddp/src/search_strategy/StrategySettings.cpp
  ${OCS2}/ocs2_ddp/src/HessianCorrection.cpp)  # hessian_correction::{to,from}String, used by StrategySettings
target_include_directories(ocs2_ddp PUBLIC ${OCS2}/ocs2_ddp/include)
target_link_libraries(ocs2_ddp PUBLIC ocs2::core ocs2::oc ocs2::qp_solver ocs2_flags Eigen3::Eigen ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_ddp PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::ddp ALIAS ocs2_ddp)

# ---- 13) ocs2_sqp ----------------------------------------------------------
file(GLOB_RECURSE OCS2_SQP_SRC CONFIGURE_DEPENDS ${OCS2}/ocs2_sqp/ocs2_sqp/src/*.cpp)
add_library(ocs2_sqp STATIC ${OCS2_SQP_SRC})
target_include_directories(ocs2_sqp PUBLIC ${OCS2}/ocs2_sqp/ocs2_sqp/include)
target_link_libraries(ocs2_sqp PUBLIC
  ocs2::core ocs2::mpc ocs2::oc ocs2::qp_solver ocs2::hpipm_interface ocs2_flags
  Eigen3::Eigen ${OCS2_BOOST_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ocs2_sqp PUBLIC OpenMP::OpenMP_CXX)
endif()
add_library(ocs2::sqp ALIAS ocs2_sqp)

# ---- solve milestone -------------------------------------------------------
# Proves the SQP+HPIPM stack RUNS end-to-end (the real BLASFEO/HPIPM-tag test).
# Uses ocs2_oc's header-only circular-kinematics OCP (in ocs2_oc/test/include).
add_executable(ocs2SolveCheck ${CMAKE_SOURCE_DIR}/tools/ocs2SolveCheck.cpp)
target_include_directories(ocs2SolveCheck PRIVATE ${OCS2}/ocs2_oc/test/include)
target_link_libraries(ocs2SolveCheck PRIVATE ocs2::sqp ocs2::oc ocs2::core ocs2_flags)

# =============================================================================
# wb's OWN MPC packages, de-ROS'd into standalone static libs. Both are ~ROS2-free
# in their src/*.cpp; the only ROS2 surface is gated by HUMANOID_MPC_NO_ROS2 (a
# macro this build defines; the ament build never does, so it stays byte-identical):
#   - common: WalkingVelocityCommand.h (msg-conversion fn), GaitScheduleUpdater.h
#     (vestigial rclcpp include) -- both guarded in-place.
#   - centroidal: mrt/CentroidalMpcMrtJointController's only ROS2 dep is the optional
#     DummyObserver viz, also gated by HUMANOID_MPC_NO_ROS2 (forward-declared there).
# =============================================================================

# ---- 14) humanoid_common_mpc (shared MPC base) -----------------------------
set(HCM ${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc)
file(GLOB_RECURSE HCM_SRC CONFIGURE_DEPENDS ${HCM}/src/*.cpp)
add_library(humanoid_common_mpc STATIC ${HCM_SRC})
target_include_directories(humanoid_common_mpc PUBLIC ${HCM}/include)
target_compile_definitions(humanoid_common_mpc PUBLIC HUMANOID_MPC_NO_ROS2)
target_link_libraries(humanoid_common_mpc PUBLIC
  ocs2::core ocs2::oc ocs2::mpc ocs2::robotic_tools ocs2::pinocchio_interface ocs2_flags
  PkgConfig::pinocchio Eigen3::Eigen ${OCS2_BOOST_LIBS})
add_library(humanoid::common_mpc ALIAS humanoid_common_mpc)

# ---- 15) humanoid_centroidal_mpc (centroidal MPC incl. mrt/ runtime bridge) -
# Full src incl. mrt/CentroidalMpcMrtJointController: it bridges the MPC policy to
# robot joint torques (inverse dynamics) and is reused by the B1 closed-loop harness.
# Its only ROS2 dep (DummyObserver viz) is gated by HUMANOID_MPC_NO_ROS2; it also pulls
# robot::model (RobotState/ControllerBase) -> requires the unified C++20 build (robot_model
# headers use concepts/span; the controller also uses std::jthread).
set(HCMPC ${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_centroidal_mpc)
file(GLOB_RECURSE HCMPC_SRC CONFIGURE_DEPENDS ${HCMPC}/src/*.cpp)
add_library(humanoid_centroidal_mpc STATIC ${HCMPC_SRC})
target_include_directories(humanoid_centroidal_mpc PUBLIC ${HCMPC}/include)
target_compile_definitions(humanoid_centroidal_mpc PUBLIC HUMANOID_MPC_NO_ROS2)
target_link_libraries(humanoid_centroidal_mpc PUBLIC
  humanoid::common_mpc robot::model
  ocs2::core ocs2::oc ocs2::mpc ocs2::ddp ocs2::sqp ocs2::centroidal_model
  ocs2::robotic_tools ocs2::pinocchio_interface ocs2_flags
  PkgConfig::pinocchio Eigen3::Eigen ${OCS2_BOOST_LIBS})
add_library(humanoid::centroidal_mpc ALIAS humanoid_centroidal_mpc)

# ---- G1 centroidal-MPC solve check (B1 kickoff / definitive 0b proof) -------
# Instantiates the real G1 centroidal MPC from the G1 config + URDF and runs one
# SQP solve. First run triggers G1 CppAD codegen (~5-15 min, cached after). Config
# paths baked in (source tree) but argv-overridable.
set(G1CFG ${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_centroidal_mpc/config)
add_executable(g1CentroidalSolveCheck ${CMAKE_SOURCE_DIR}/tools/g1CentroidalSolveCheck.cpp)
target_link_libraries(g1CentroidalSolveCheck PRIVATE humanoid::centroidal_mpc humanoid::common_mpc ocs2::sqp ocs2_flags)
target_compile_definitions(g1CentroidalSolveCheck PRIVATE
  G1_TASK_FILE="${G1CFG}/mpc/task.info"
  G1_REFERENCE_FILE="${G1CFG}/command/reference.info"
  G1_GAIT_FILE="${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc/config/command/gait.info"
  G1_URDF_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf")
set_target_properties(g1CentroidalSolveCheck PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- B1 standing closed-loop harness (centroidal MPC closed on the MuJoCo G1) ----
# Non-ROS2 CentroidalMpcRobotSim: drives the real MPC through the MRT controller on
# the headless MuJoCo sim and checks the G1 stays upright. Needs both the MPC libs
# (this file) and robot::mujoco_sim_interface (root CMakeLists, defined before this
# include). Same G1 config paths as the solve check + the MuJoCo scene xml.
add_executable(standingClosedLoop ${CMAKE_SOURCE_DIR}/experiments/standingClosedLoop.cpp)
target_link_libraries(standingClosedLoop PRIVATE
  humanoid::centroidal_mpc humanoid::common_mpc robot::mujoco_sim_interface ocs2::sqp ocs2_flags)
target_compile_definitions(standingClosedLoop PRIVATE
  G1_TASK_FILE="${G1CFG}/mpc/task.info"
  G1_REFERENCE_FILE="${G1CFG}/command/reference.info"
  G1_GAIT_FILE="${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc/config/command/gait.info"
  G1_URDF_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf"
  G1_SCENE_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml")
set_target_properties(standingClosedLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- B1 push-recovery harness (scripted xfrc disturbance on the closed-loop G1) ----
# Same wiring as standingClosedLoop + MujocoSimInterface::setExternalWrench push.
add_executable(pushRecovery ${CMAKE_SOURCE_DIR}/experiments/pushRecovery.cpp)
target_link_libraries(pushRecovery PRIVATE
  humanoid::centroidal_mpc humanoid::common_mpc humanoid::adr robot::mujoco_sim_interface ocs2::sqp ocs2_flags)
target_compile_definitions(pushRecovery PRIVATE
  G1_TASK_FILE="${G1CFG}/mpc/task.info"
  G1_REFERENCE_FILE="${G1CFG}/command/reference.info"
  G1_GAIT_FILE="${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc/config/command/gait.info"
  G1_URDF_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf"
  G1_SCENE_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml")
set_target_properties(pushRecovery PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- B3 rung 1: footstep-policy plumbing probe (ReactiveStepper, scripted single step) ----
# Standing closed loop + a scripted gait injection; verifies the foot lifts/replants and the
# robot stays up before any capture-point trigger is wired in. No observer (no humanoid::adr).
add_executable(stepProbe ${CMAKE_SOURCE_DIR}/experiments/stepProbe.cpp)
target_link_libraries(stepProbe PRIVATE
  humanoid::centroidal_mpc humanoid::common_mpc robot::mujoco_sim_interface ocs2::sqp ocs2_flags)
target_compile_definitions(stepProbe PRIVATE
  G1_TASK_FILE="${G1CFG}/mpc/task.info"
  G1_REFERENCE_FILE="${G1CFG}/command/reference.info"
  G1_GAIT_FILE="${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc/config/command/gait.info"
  G1_URDF_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf"
  G1_SCENE_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml")
set_target_properties(stepProbe PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- B5: integrate observer feedforward (B2) + reactive stepping (B3) in one harness ----
# stepProbe wiring + the observer/feedforward block from pushRecovery (needs humanoid::adr).
add_executable(b5Probe ${CMAKE_SOURCE_DIR}/experiments/b5Probe.cpp)
target_link_libraries(b5Probe PRIVATE
  humanoid::centroidal_mpc humanoid::common_mpc humanoid::adr robot::mujoco_sim_interface ocs2::sqp ocs2_flags)
target_compile_definitions(b5Probe PRIVATE
  G1_TASK_FILE="${G1CFG}/mpc/task.info"
  G1_REFERENCE_FILE="${G1CFG}/command/reference.info"
  G1_GAIT_FILE="${CMAKE_SOURCE_DIR}/humanoid_nmpc/humanoid_common_mpc/config/command/gait.info"
  G1_URDF_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf"
  G1_SCENE_FILE="${CMAKE_SOURCE_DIR}/robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml")
set_target_properties(b5Probe PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
