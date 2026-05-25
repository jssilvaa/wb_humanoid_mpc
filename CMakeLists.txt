# =============================================================================
# STANDALONE (non-ROS 2) build for the active-disturbance-rejection contribution.
#
# This is SEPARATE from the ROS 2 / colcon build (driven by the Makefile). It is
# NON-DESTRUCTIVE: it compiles the existing package sources directly and never
# invokes the per-package ament CMakeLists, which stay intact for colcon.
# Driven by ./build.sh. See DISTURBANCE_REJECTION_PLAN.md (Phase 0).
#
# Phase 0a scope: MuJoCo + robot_runtime so the G1 stands in MuJoCo standalone.
# (OCS2 / centroidal MPC = Phase 0b, added later for the ViC track.)
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(wb_humanoid_mpc_standalone CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# RPATH so binaries find FetchContent'd MuJoCo without DYLD/LD_LIBRARY_PATH.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
  list(APPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
endif()

# -----------------------------------------------------------------------------
# MuJoCo: FetchContent by default; -DMUJOCO_DIR=... for a prebuilt release.
# We use robot_runtime's own MujocoRenderer (raw GLFW), so libmujoco_simulate is
# not needed -> BUILD_SIMULATE OFF (faster build).
# -----------------------------------------------------------------------------
option(FETCH_MUJOCO "Build MuJoCo from source via FetchContent" ON)
if(FETCH_MUJOCO)
  include(FetchContent)
  FetchContent_Declare(mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        3.8.0)
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(mujoco)
else()
  if(NOT DEFINED MUJOCO_DIR)
    message(FATAL_ERROR "Set -DMUJOCO_DIR=/path/to/mujoco or pass -DFETCH_MUJOCO=ON.")
  endif()
  add_library(mujoco::mujoco SHARED IMPORTED)
  if(APPLE)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION             "${MUJOCO_DIR}/lib/libmujoco.3.8.0.dylib"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include")
  else()
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION             "${MUJOCO_DIR}/lib/libmujoco.so.3.8.0"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include")
  endif()
endif()

# MuJoCo's FetchContent path may already create Eigen3::Eigen.
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED NO_MODULE)
endif()

find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(absl REQUIRED)
if(NOT APPLE)
  find_package(GLEW REQUIRED)
endif()

# urdfdom (brewed): config-file based; the brew install does not expose its cmake
# config via CMAKE_PREFIX_PATH by default, so hint it. (Recipe from vic-mpc.)
find_package(urdfdom CONFIG REQUIRED
  HINTS /opt/homebrew/opt/urdfdom/lib/urdfdom/cmake
        /usr/local/opt/urdfdom/lib/urdfdom/cmake)
find_package(urdfdom_headers CONFIG REQUIRED
  HINTS /opt/homebrew/opt/urdfdom_headers/lib/urdfdom_headers/cmake
        /usr/local/opt/urdfdom_headers/lib/urdfdom_headers/cmake)
# The reference includes parser headers as <urdfdom/urdf_parser/...>, so put the
# package prefix include on the path too (derived from the *_DIR cache vars).
set(URDFDOM_PREFIX_INCLUDE         "${urdfdom_DIR}/../../../include")
set(URDFDOM_HEADERS_PREFIX_INCLUDE "${urdfdom_headers_DIR}/../../../include")

# Pinocchio (brewed) via pkg-config — composite-rigid-body / centroidal algorithms.
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED IMPORTED_TARGET pinocchio)

set(RR ${CMAKE_CURRENT_SOURCE_DIR}/robot_runtime)

# -----------------------------------------------------------------------------
# robot_core (header-only)
# -----------------------------------------------------------------------------
add_library(robot_core INTERFACE)
target_include_directories(robot_core INTERFACE ${RR}/robot_core/include)
target_link_libraries(robot_core INTERFACE Eigen3::Eigen)
add_library(robot::core ALIAS robot_core)

# -----------------------------------------------------------------------------
# robot_model (urdfdom-based RobotDescription + RobotState)
# -----------------------------------------------------------------------------
add_library(robot_model STATIC
  ${RR}/robot_model/src/RobotDescription.cpp
  ${RR}/robot_model/src/RobotState.cpp)
target_include_directories(robot_model PUBLIC
  ${RR}/robot_model/include
  ${URDFDOM_PREFIX_INCLUDE}         ${URDFDOM_PREFIX_INCLUDE}/urdfdom
  ${URDFDOM_HEADERS_PREFIX_INCLUDE} ${URDFDOM_HEADERS_PREFIX_INCLUDE}/urdfdom_headers)
target_link_libraries(robot_model PUBLIC
  robot::core
  ${urdfdom_LIBRARIES}
  absl::flat_hash_map)
add_library(robot::model ALIAS robot_model)

# -----------------------------------------------------------------------------
# mujoco_sim_interface (MuJoCo physics + raw-GLFW renderer)
# -----------------------------------------------------------------------------
add_library(mujoco_sim_interface STATIC
  ${RR}/mujoco_sim_interface/src/MujocoSimInterface.cpp
  ${RR}/mujoco_sim_interface/src/MujocoRenderer.cpp)
target_include_directories(mujoco_sim_interface PUBLIC ${RR}/mujoco_sim_interface/include)
target_link_libraries(mujoco_sim_interface PUBLIC
  robot::model
  robot::core
  mujoco::mujoco
  glfw
  Threads::Threads)
# NOTE: the ROS 2 CMakeLists linked Boost (system/filesystem/log) + absl here,
# but the mujoco_sim_interface sources reference neither (verified). robot::model
# carries absl transitively for the headers that need it.
if(APPLE)
  target_link_libraries(mujoco_sim_interface PUBLIC
    "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
else()
  target_link_libraries(mujoco_sim_interface PUBLIC GLEW::GLEW)
endif()
add_library(robot::mujoco_sim_interface ALIAS mujoco_sim_interface)

# -----------------------------------------------------------------------------
# Phase 0a deliverable: hold the G1 standing in MuJoCo (PD around default pose).
# -----------------------------------------------------------------------------
add_executable(mujocoSimPDStand
  ${RR}/mujoco_sim_interface/exe/mujocoSimPDStand.cpp)
target_link_libraries(mujocoSimPDStand PRIVATE robot::mujoco_sim_interface)
set_target_properties(mujocoSimPDStand PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# -----------------------------------------------------------------------------
# humanoid_disturbance_rejection: the ADR contribution. OCS2-independent.
# A1: the cascaded centroidal-momentum wrench observer (pure Eigen).
# (A1.2 will add a Pinocchio CCRBA helper for h/c_G.)
# -----------------------------------------------------------------------------
set(HDR ${CMAKE_CURRENT_SOURCE_DIR}/humanoid_nmpc/humanoid_disturbance_rejection)
add_library(humanoid_disturbance_rejection STATIC
  ${HDR}/src/CentroidalMomentumObserver.cpp
  ${HDR}/src/CentroidalMomentumCalculator.cpp)
target_include_directories(humanoid_disturbance_rejection PUBLIC ${HDR}/include)
target_link_libraries(humanoid_disturbance_rejection PUBLIC Eigen3::Eigen PkgConfig::pinocchio)
# Pinocchio headers pull in Boost; on Homebrew (Apple Silicon) Boost sits under the
# brew prefix, which Apple clang does not search by default. Add it explicitly.
if(APPLE AND EXISTS /opt/homebrew/include)
  target_include_directories(humanoid_disturbance_rejection SYSTEM PUBLIC /opt/homebrew/include)
endif()
add_library(humanoid::adr ALIAS humanoid_disturbance_rejection)

# A1.1 verification: synthetic step-disturbance convergence + detection latency.
add_executable(observerSelfCheck ${HDR}/test/observerSelfCheck.cpp)
target_link_libraries(observerSelfCheck PRIVATE humanoid::adr)
set_target_properties(observerSelfCheck PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# A1.2 verification: Pinocchio centroidal-momentum helper (h, com from q, v).
add_executable(centroidalSanityCheck ${HDR}/test/centroidalSanityCheck.cpp)
target_link_libraries(centroidalSanityCheck PRIVATE humanoid::adr)
set_target_properties(centroidalSanityCheck PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
