1. Ask, don't assume. If something is unclear, ask before writing a single line. Never make silent assumptions about intent, architecture, or requirements.

2. Simplest solution first. Always implement the simplest thing that could work. Do not add abstractions or flexibility that weren't explicitly requested.

3. Don't touch unrelated code. If a file or function is not directly part of the current task, do not modify it, even if you think it could be improved.

4. Flag uncertainty explicitly. If you are not confident about an approach or technical detail, say so before proceeding. Confidence without certainty causes more damage than admitting a gap.

---

# Project Knowledge Base

> Appended by Claude (2026-05-25). The 4 rules above are the user's working rules — do not edit them. Everything below is a reference map of the repo + the OCS2 library, plus the research objective of this fork. Items marked **(?)** are hypotheses still to be confirmed against `/Users/josesilvaa/control-rejection`.

## 1. What this repo is

`wb_humanoid_mpc` — Whole-Body **Nonlinear MPC** for humanoid loco-manipulation, targeting the **Unitree G1**. Forked from `manumerous/wb_humanoid_mpc` (M. Galliker / 1X). Built on an extended **OCS2** (`manumerous/ocs2_ros2`, vendored as a submodule at `lib/ocs2_ros2`). It is a **ROS 2 (Jazzy) colcon workspace**. Heavy build (RAM-bound; first run does CppAD code-gen, 5–15 min).

Two hardware-agnostic MPC formulations:
- **Centroidal Dynamics MPC** (`humanoid_centroidal_mpc`): optimizes whole-body kinematics + CoM/centroidal dynamics. Supports full centroidal dynamics **or** SRBD. Builds on OCS2's centroidal model, generalized to **6-DoF contacts**.
- **Whole-Body Dynamics MPC** (`humanoid_wb_mpc`): optimizes over contact forces + joint accelerations (acceleration/torque level).

## 2. Research objective of THIS fork (J. Silva, PI / research project)

Port two **active-disturbance-rejection** components — developed/prototyped in `/Users/josesilvaa/control-rejection` — into this control stack, then measure efficacy against disturbances and observer accuracy, and document in `control-rejection/relatorio-pi/`:
1. **6D external-wrench observer** — a centroidal-momentum observer estimating the external wrench acting on the robot (the "easy" / working piece).
2. **ViC-MPC (Variable-Inertia Centroidal MPC)** — centroidal MPC where the inertia is variable/state-dependent for better disturbance rejection (the messy / not-yet-working prototype).

See memory files for prior-work locations and intent. Planning is tracked in the task list (`TaskList`).

## 3. Repo layout

| Path | Role |
|---|---|
| `humanoid_nmpc/humanoid_common_mpc` | Shared MPC base: `MpcRobotModelBase` (state/input indexing), gait + swing planning, `ContactWrenchMapper`, generic costs/constraints (`FrictionForceConeConstraint`, `ZeroWrenchConstraint`, `ExternalTorqueQuadraticCostAD`, …), `HumanoidCostConstraintFactory`, `HumanoidPreComputation`, `SwitchedModelReferenceManager`, `ProceduralMpcMotionManager`, pinocchio model helpers |
| `humanoid_nmpc/humanoid_centroidal_mpc` | Centroidal MPC: **`CentroidalMpcInterface`** (assembles the OCP), **`dynamics/CentroidalDynamicsAD`** (wraps OCS2 `PinocchioCentroidalDynamicsAD`), `common/CentroidalMpcRobotModel`, costs (`ICPCost`, `CentroidalMpcEndEffectorFootCost`), constraints (Zero/NormalVelocity, JointMimic), `mrt/CentroidalMpcMrtJointController` |
| `humanoid_nmpc/humanoid_wb_mpc` | Whole-Body MPC: `WBMpcInterface`, `dynamics/WBAccelDynamicsAD`, torque cost, EE-dynamics costs/constraints, `mrt/WBMpcMrtJointController` |
| `humanoid_nmpc/*_ros2` | ROS 2 nodes/wrappers. Centroidal: `CentroidalMpcSqpNode` (solver), `CentroidalMpcDummySimNode`, `CentroidalMpcRobotSim`, live-tuning `gains/` framework. Common: `MRTPolicySubscriber`, visualization (`HumanoidVisualizer`, `EquivalentContactCornerForcesVisualizer`), DDP/SQP benchmark publishers |
| `humanoid_nmpc/humanoid_mpc_msgs` | Custom ROS 2 messages |
| `humanoid_nmpc/remote_control` | Joystick / GUI base-velocity + height commands |
| `robot_models/unitree_g1` | `g1_description` (URDF/meshes), `g1_centroidal_mpc` (config/launch), `g1_wb_mpc` (config/launch) |
| `robot_runtime` | `mujoco_sim_interface` (MuJoCo sim + renderer), `robot_model` (`RobotDescription`, `RobotState`), `robot_core` |
| `lib` | Submodules: `ocs2_ros2`, `mujoco`, `magic_enum`, `catkin`, `cmake_modules` |

**Runtime data flow (MPC ↔ MRT split):** `CentroidalMpcSqpNode` solves the OCP in real time and publishes the policy → MRT side (`CentroidalMpcMrtJointController` + `CentroidalMpcRobotSim`/`DummySimNode`) evaluates the policy, drives the MuJoCo sim, and feeds measured state back to the MPC. Gains/weights are live-tunable from a GUI via `gains/` + `GainsReceiver`.

**Build / run:** `make build-all` (set `PARALLEL_JOBS`, RAM-bound) · `make launch-g1-dummy-sim` (centroidal) · `make launch-wb-g1-dummy-sim` (whole-body).

## 4. OCS2 essentials (the library to know)

OCS2 = Optimal Control for Switched Systems. Solvers: **SLQ** (cont. DDP), **iLQR** (disc. DDP), **SQP** (multiple-shooting on HPIPM — *this repo uses SQP*), **SLP** (PIPG), **IPM** (interior point). Path constraints via augmented-Lagrangian / relaxed-barrier.

Core abstraction — `OptimalControlProblem` = a collection of named terms, each evaluated at **intermediate / prejump / final** times:
- **Costs**: `StateCost`, `StateInputCost` (+ AD: `StateCostCppAd`, `StateInputCostCppAd`, `StateInputCostGaussNewtonAd`). Solvers want PD/PSD Hessians.
- **Constraints**: `StateConstraint`, `StateInputConstraint` (+ CppAd). Hard (state-input equality via projection — needs full row-rank input Jacobian; state eq/ineq via barrier/AL) or **soft** (penalty wrapping via `StateSoftConstraint`/`StateInputSoftConstraint`).
- **Dynamics**: `SystemDynamicsBase` (+ `SystemDynamicsBaseAD`) — flow-map, jump-map, first-order approx.
- **PreComputation**: cache shared between cost/constraint/dynamics; `request()` gated by request flags.

**Changing parameters at runtime (the injection path):** the solver copies the OCP internally, so you must NOT mutate terms directly. Use:
- **`ReferenceManagerInterface`** (`ReferenceManager`, `ReferenceManagerRos`, here `SwitchedModelReferenceManager`): target trajectories + mode schedule; `preSolverRun()` runs before each MPC iteration.
- **`SolverSynchronizedModule`**: `preSolverRun(initTime, finalTime, initState, referenceManager)` and `postSolverRun(primalSolution)` — general-purpose pre/post hooks. **This is where an external-wrench estimate gets injected into the MPC.** Keep these light; offload to a worker thread + `BufferedValue` (address swap).

**Centroidal model (`lib/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model`) — central to both research components:**
- `CentroidalModelType` ∈ { `FullCentroidalDynamics`, `SingleRigidBodyDynamics` }.
- `CentroidalModelInfo`: `robotMass`, `qPinocchioNominal`, **`centroidalInertiaNominal`** (constant inertia for SRBD — the likely ViC variable **(?)**), `comToBasePositionNominal`, dims, 3-DoF/6-DoF contact counts, EE frame indices.
- `PinocchioCentroidalDynamics` / `…AD` — the flow map:
  - **State** `x = [ h_lin/m, h_ang/m, p_base(3), θ_base(ZYX,3), q_joints ]` — first 6 = normalized centroidal momentum (centroidal frame = at CoM, inertial-aligned).
  - **Input** `u = [ contact_forces(3·n3dof), contact_wrenches(6·n6dof), joint_velocities ]` — forces/wrenches in inertial frame.
  - Core internal: `computeNormalizedCentroidalMomentumRate(...)` — the momentum-rate ODE. **External wrench feedforward and the observer's residual model both live on this equation.**
- Helpers: `CentroidalModelPinocchioMapping` (centroidal ↔ pinocchio v), `CentroidalModelRbdConversions`, `ModelHelperFunctions` (CCRBA / centroidal momentum matrix), `AccessHelperFunctions` (slice momentum/pose/joints out of `x`), `FactoryFunctions` (build from URDF).

**OCS2 docs (read for future reference):** `lib/ocs2_ros2/ocs2_doc/docs/*.rst` — `overview`, `optimal_control_modules` (cost/constraint/dynamics/precomp + ReferenceManager/SolverSynchronizedModule), `from_urdf_to_ocp` (centroidal model, Pinocchio kinematics, self-collision), `getting-started`, `robotic_examples`, `installation`, `mpcnet`, `profiling`, `faq`; bibliography in `refs.bib`. Background paper: Sleiman et al., *A Unified MPC Framework for WB Dynamic Locomotion and Manipulation* (centroidal model).

## 5. Integration points for the research components (working hypotheses — confirm in planning)

- **6D wrench observer.** Inputs: measured state (base pose/twist, joints) + measured/estimated contact wrenches (sim provides ground-truth forces; visualized via `EquivalentContactCornerForcesVisualizer`). Model: residual between measured normalized-centroidal-momentum rate and the model prediction from contact wrenches ⇒ estimated external 6D wrench. Likely homes: (a) a standalone ROS 2 node on the MRT/robot side, or (b) a `SolverSynchronizedModule`/runtime component. Output published for evaluation, and optionally fed back into the MPC as an external-wrench parameter on the momentum dynamics.
- **ViC-MPC.** Make the centroidal inertia state/configuration-dependent rather than the nominal constant (`centroidalInertiaNominal`, SRBD) **(?)**. Candidate sites: a custom dynamics replacing/extending `CentroidalDynamicsAD` / `PinocchioCentroidalDynamicsAD`, `CentroidalModelInfo`, OCP assembly in `CentroidalMpcInterface::setupOptimalControlProblem`, and possibly cost shaping (`ICPCost`). Evaluate against the observer + disturbance scenarios.

## 6. Decided architecture (2026-05-25)

Integration approach chosen by José:
- **Implement in C++**, as a **standalone (non-ROS 2) build inside this fork** (`wb_humanoid_mpc/`, on a feature branch — this is "our contribution"). The `_ros2` packages are NOT the integration surface; RViz/gamepad may be reused opportunistically.
- **`/Users/josesilvaa/vic-mpc/`** is José's scratch study of the repo: a standalone non-ROS 2 build (bundled HPIPM/BLASFEO/OCS2 + C++ MuJoCo sim + G1 AD codegen, `_ros2` stripped). Use it as the **reference recipe** for the standalone build; the contribution lands here in-tree.
- **Observer**: port the working Python prototype (`control-rejection/heng/src/centroidal_observer.py` + `dynamics.py`/`jacobians.py`) to C++; evaluate in the C++ MuJoCo sim with applied disturbances (`xfrc_applied`; ground truth via `xipos`). Python kept only for plots/analysis.
- **ViC-MPC / active disturbance rejection — a contribution TO OCS2, not a separate controller** (José, 2026-05-25). Port the *disturbance-rejection framework* into the existing OCS2 centroidal MPC, **reusing OCS2's existing locomotion machinery** (`SwitchedModelReferenceManager`, `GaitSchedule`/`ModeSequenceTemplate`/`MotionPhaseDefinition`, `SwingTrajectoryPlanner`, `*TargetTrajectoriesCalculator`) rather than re-porting Meng's Python stepping. The *novel blocks* are: (a) **configuration-dependent (variable) centroidal inertia** in the dynamics, and (b) **ADR** = observer `Ŵ` fed forward into the centroidal momentum-rate equation + capturability/DCM-driven footstep adaptation. OCS2 supplies the walking substrate.
- **Phased, verify each block:** P1 = variable-inertia dynamics + small disturbances, balance regime (no stepping change), verify; P2+ = disturbance feedforward/rejection blocks, verify each; **final (biggest contribution) = stepping-based disturbance rejection** on top of OCS2's stepping. Observer and ViC start **decoupled** (no `Ŵ`→QP injection — report §07); live coupling is a later phase ("if we can integrate it, all the better").
- **Shared eval dependency**: applying the centroidal MPC policy on the full MuJoCo G1 (whole-body tracking) — verify what `robot_runtime/mujoco_sim_interface` + `CentroidalMpcMrtJointController` already provide (observer eval avoids this via position-hold; closed-loop ViC needs it).
