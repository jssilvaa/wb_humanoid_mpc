# Active Disturbance Rejection for the OCS2 Centroidal MPC — Implementation Plan

> Working plan for José's research contribution (PI report). Drafted 2026-05-25.
> Companion to `CLAUDE.md` ("Project Knowledge Base" §1–§6). Math source of truth = `control-rejection/relatorio-pi/`.

## Goal & guiding decisions

Bring a **6D centroidal-momentum external-wrench observer** and a **variable-inertia + active-disturbance-rejection (ADR) framework** into this fork's OCS2 centroidal MPC, and measure both against disturbances in MuJoCo so results can be documented in `relatorio-pi/`.

- **C++**, **standalone (non-ROS 2) build**, **in-tree in this fork** (feature branch = our contribution). `vic-mpc/` is the *reference recipe* for the standalone build, not the target.
- **Contribute to OCS2**: reuse its existing locomotion (gait/mode schedule, phase transitions, footstep + swing planners). The *novel blocks* are the **observer**, the **variable centroidal inertia**, and the **disturbance feedforward + step adaptation**.
- **Phased, verify each block** before the next. Each gate produces a report-able result.
- **Eval** per report §07 (MuJoCo G1, Δt=1 ms; scenarios, KPIs, validity guards). **Python only for plots/analysis.**
- Observer and ViC start **decoupled** (no `Ŵ` → MPC), matching §07; live coupling is an explicit later phase.

## Components (target architecture)

```
            ┌─────────────────────── standalone C++ apps (no ROS 2) ───────────────────────┐
 MuJoCo G1  │  observer_harness (balance)            closed_loop_harness (ViC)              │
 sim (1ms)  │  position-hold + xfrc push             OCS2 Centroidal MPC ── policy ──┐      │
   ▲   │    │        │                                  ▲          │                  ▼      │
   │   ▼    │   [Observer C++]──Ŵ              state ───┘   [+var inertia]   CentroidalMpc   │
 contacts,  │        │  (CMM h,c via Pinocchio;          [+Ŵ feedforward]    MrtJointCtrl    │
 xipos      │   compare Ŵ vs W_true)          [+DCM step adaptation]  (invdyn+impedance)→τ  │
            └──────────────────────────────────────────────────────────────────────────────┘
 reused (existing): OCS2 centroidal MPC, gait/swing/reference machinery, MRT controller, mujoco_sim_interface
 new: Observer lib · variable-inertia precompute · wrench-feedforward dynamics wrap · step-adaptation SolverSynchronizedModule · logging/metrics
```

---

## Phase 0 — Standalone (non-ROS 2) build  *(foundation; authored in-tree, copying vic-mpc's recipe — Claude writes, José reviews)*

**Toolchain confirmed on the dev Mac:** cmake/ninja/clang, Eigen, Pinocchio, CppADCodeGen, HPIPM all build standalone (no ROS 2/colcon). The ROS 2 colcon build does NOT run here — hence standalone. `vic-mpc/` is a WIP reference recipe, not a dependency.

**Staged:**
- **0a — MuJoCo + Pinocchio + G1 stand (unblocks Track A).** Standalone CMake: MuJoCo via FetchContent (3.8.0) + Pinocchio/Eigen (system); port `robot_runtime/{robot_core,robot_model,mujoco_sim_interface}` to plain CMake (vic-mpc's CMake is the template); a `mujocoSimPDStand`-style exe holding the G1 in MuJoCo. **No OCS2 needed** — the observer only needs MuJoCo (sim+contacts) + Pinocchio (CCRBA) + Eigen. Gate: G1 stands in MuJoCo from a standalone binary here.
- **0b — Full OCS2 SQP standalone (needed for Track B).** Plain-CMake the OCS2 subset incl. the solver: `ocs2_thirdparty` (HPIPM/BLASFEO) → `ocs2_core` (+CppADCodeGen) → `ocs2_oc` → `ocs2_robotic_tools` → `ocs2_pinocchio_interface` → `ocs2_centroidal_model` → `ocs2_sqp`/`ocs2_mpc`/`ocs2_ddp`; then `humanoid_common_mpc` → `humanoid_centroidal_mpc`. This is MORE than vic-mpc did (it hand-rolled SQP, deferred CppAD) — the main build-up. Gate: standalone centroidal MPC solves + drives the MuJoCo G1 (reusing `CentroidalMpcMrtJointController`).
- **Risk:** 0b's ament→plain-CMake of `ocs2_sqp` + `ocs2_thirdparty` + the CppAD path is the principal unknown; build bottom-up with a compile check at each package.

### Phase 0b — detailed build ladder (mapped 2026-05-25)

Built in `cmake/Ocs2Standalone.cmake`, gated by `-DBUILD_OCS2=ON`, as plain-CMake **static** libs from wb's **full** `lib/ocs2_ros2` sources (`file(GLOB src/*.cpp)`). vic-mpc's `lib/ocs2` CMakeLists are reused as the **find-pattern** only (vic stripped CppAD + stubbed `oc`).

Bottom-up order (compile-gate each): **1** `ocs2_thirdparty` (hdr-only: bundled CppAD + CppADCodeGen + iit) → **2** `ocs2_core` (85 src — *risk gate 1*) → **3** `ocs2_oc` (full, ~30 src — *risk gate 2*, vic only stubbed it) → **4** `ocs2_robotic_tools` → **5** `ocs2_pinocchio_interface` (reuse vic's brew pinocchio/urdfdom recipe) → **6** `ocs2_centroidal_model` → **7** `ocs2_qp_solver` (dense KKT, no HPIPM) → **8/9** `blasfeo`+`hpipm` (reuse vic `external/install`, `TARGET=GENERIC`, link `-lhpipm -lblasfeo -lm`) → **10** `hpipm_catkin` (`HpipmInterface.cpp`) → **11** `ocs2_mpc` → **12** `ocs2_ddp` → **13** `ocs2_sqp` (`SqpMpc` is header-only) → **milestone:** tiny main that builds a trivial `OptimalControlProblem` + `SqpMpc` solve → **14** `humanoid_common_mpc` → **15** `humanoid_centroidal_mpc` (de-ROS: strip `rclcpp`/`ament_*`, replace `humanoid_mpc_msgs`/`ocs2_ros2_interfaces` with plain C++).

**Build gotchas (macOS/clang/C++20):** do NOT use `ocs2_core/cmake/ocs2_cxx_flags.cmake` (forces C++14 + `-Wl,--no-as-needed`). Use C++20 + `-Wno-invalid-partial-specialization` + `-DBOOST_MPL_LIMIT_LIST_SIZE=30`. Brew Boost 1.90 component config is broken → `find_library(boost_{system,filesystem,log,log_setup})` + `/opt/homebrew/include` for headers. CppAD codegen → `.dylib` → `dlopen` validated (ClangCompiler, `-ldl`).

**Status (2026-05-25) — compile + generic-solve DONE; G1 instantiate+solve = B1 kickoff.** Built in `cmake/Ocs2Standalone.cmake`. Deviations from the plan above, as actually built:
- **C++17, not C++20** for the OCS2 side: `std::result_of` (used by `LinearInterpolation.h`) was removed in C++20. `ocs2_flags` = `-std=gnu++17 -Wno-invalid-partial-specialization -include cassert -O2`. (Observer/MuJoCo side stays C++20; C++17↔C++20 static libs link fine.)
- **BLASFEO+HPIPM built FROM SOURCE at OCS2's pinned tags** (`build_hpipm.sh` → `external/install`), **not** vic's prebuilt — vic's tag mismatched `d_ocp_qp_dim_set_all`. Needs `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` (CMake 4 dropped `cmake_minimum_required<3.5`).
- **2 portable clang patches** in the `lib/ocs2_ros2` submodule (tracked in `OCS2_CLANG_PATCHES.md`, left uncommitted there): `MultidimensionalPenalty.cpp` (drop ctor template args), `SqpSolver.cpp` (`to_time_t` on `system_clock`, not `high_resolution_clock`).
- **`ocs2_ddp` = settings-only** (`DDP_Settings.cpp` alone): the DDP solver proper (`DDP_DataCollector`) references OCS2 API removed in this version; we solve with SQP and only need `ddp::Settings`.
- **Milestone** = `tools/ocs2SolveCheck.cpp` (OCS2 circular-kinematics OCP; PASS, violations ≈1e-32/1e-21).
- **de-ROS via `HUMANOID_MPC_NO_ROS2` macro** (the standalone build defines it; ament never does → that build stays byte-identical): guards 2 headers (`WalkingVelocityCommand.h` msg-conversion fn, `GaitScheduleUpdater.h` vestigial rclcpp). `humanoid_centroidal_mpc` excludes `mrt/` (ROS2/robot_runtime runtime bridge). The MRT MuJoCo-drive gate moves to B1.

---

## Track A — 6D Centroidal Wrench Observer  *(Task 6)*

### A1 — C++ observer library
- **Goal:** port `heng/src/centroidal_observer.py` (the cascade + `W_known` + CMM) to C++.
- **Build:** `humanoid_common_mpc/observer/CentroidalMomentumObserver.{h,cpp}` — order-`r` cascade, gains `K=[α/3, α, 3α]`, forward-Euler `dt`; profiles (SHORT_PULSE α=70 default, AGILE40, FAST, STABLE). Get `h` (centroidal momentum) and `c_G` (CoM) from **Pinocchio centroidal** (`ocs2_centroidal_model` / CCRBA: `data.hg`, `data.com[0]`, `data.Ag`) — reuse the model we already build, not MuJoCo's CMM. `W_known` = `m·g` + Σ contact wrenches about CoM. **Convention: `[f; τ]` / `[lin; ang]`** (matches OCS2 + report; assert it — avoids the Π footgun).
- **Verify (gate):** analytic checks (pole placement; step response `t₃₀ = 1.9138/α`) + a sanity run where `Ŵ` responds to a known applied push. **No C++↔Python bit-parity test** (per José) — the observer is validated by the `Ŵ` trace vs the known `W_true` in A2.

### A2 — MuJoCo observer harness (balance regime)
- **Goal:** reproduce the §07 observer characterization.
- **Build:** standalone `observer_harness` — load `unitree_g1/scene.xml`, hold posture (position actuators; leg-PD kp=80/kd=2 for torso tests), apply `xfrc_applied` push at `torso_link`, run the observer each step; log `Ŵ`, `W_true` (from `xipos`), contacts, regime guards. Sweeps: long pulses (10–200 N, 0.7 s), medium pulses (0.2/0.3/0.5 s), sustained bias.
- **Verify (gate):** `Ŵ` vs known `W_true` — detection delay < 30 ms (α=70); `pulse_relerr` meets §07 (`αT ≥ 20 ⇒ ~15%`); `regime_guard_pass` filtering applied. Plots (Python, heng schema) → report §08.
- **Risk:** correctness of (i) contact-wrench aggregation (`mj_contactForce`, world-frame, GRF-only) and (ii) the momentum source (Pinocchio CCRBA `data.hg` vs MuJoCo subtree). Sanity-check both against a known static-load case early.

**Outcome:** the observer is fully validated and documentable on its own (independent of the MPC).

---

## Track B — ViC / Active Disturbance Rejection  *(Task 7, phased)*

### B1 (P1) — Variable- vs constant-inertia ablation  *(balance, small disturbances)*
- **Resolved (2026-05-25, verified in source):** OCS2 already implements both — `FullCentroidalDynamics` (config-dependent `data.Ig(q)` + limb angular momentum) vs `SingleRigidBodyDynamics` (constant `centroidalInertiaNominal`, no limb momentum), differing **only** in the inertia model (`ModelHelperFunctionsImpl.h:136-190`, `getCentroidalMomentumZyxGradient`); state, costs, constraints, SQP identical. The momentum-*rate* equation is inertia-free (= the observer's `W_known`). So "variable inertia" needs **no new dynamics code** — it's the `centroidalModelType` flag. Documented in `relatorio-pi/note_ocs2.tex`; see [[vicmpc-understanding]].
- **Goal:** quantify the variable-inertia effect on disturbance rejection as a clean controlled ablation — `Full` vs `SRBD`, all else equal.
- **Build:** expose `centroidalModelType` in the standalone harness/config; ensure both build (AD codegen per model); wire the metrics/loggers (shared with Track A). **No dynamics changes.**
- **Verify (gate):** balance regime, minor pushes (no stepping); `Full` vs `SRBD` on peak CoM/DCM deviation + recovery; confirm stability + solve rate for both. → first ViC result for the report.
- **Note:** OCS2-`Full` is richer than Meng's template (limb momentum; inertia exact along the optimized trajectory). The report's reduced ViC template (§6) maps onto OCS2-`Full` as the parametrized cousin — see `note_ocs2.tex`.

### B2 (P2) — External-wrench feedforward  *(first observer↔MPC coupling)*
- **Goal:** feed the observer `Ŵ` forward into the MPC's centroidal momentum-rate prediction (`ḣ = W_known + Ŵ`).
- **Build:** wrap `CentroidalDynamicsAD` to add the `Ŵ` term to the momentum-rate equation; supply `Ŵ` as a **time-varying parameter** updated each cycle by a `SolverSynchronizedModule` reading the observer.
- **Verify (gate):** sustained-bias and minor-push rejection improves vs B1 (peak CoM/CP deviation ↓); observer+MPC stable.
- **Uncertainty to resolve (flag):** OCS2 dynamics are CppAD-codegen'd — injecting a runtime parameter needs the parameter mechanism (or recompile). Validate the parameter path early; **fallback** = apply `Ŵ` via the reference/initialization rather than the AD flow map. (Note: this coupling goes *beyond* §07, which is decoupled — it's a new contribution.)

### B3 (final, biggest contribution) — Disturbance-driven stepping
- **Goal:** capturability/DCM-based footstep adaptation for the major (step-requiring) pushes, **reusing OCS2 stepping**.
- **Build:** a new `SolverSynchronizedModule` (before `ProceduralMpcMotionManager`) that computes `DCM = c + ċ/ω` (+ RHACV / observer signal), decides the step trigger (B2S) and an adapted foothold (DCM-landing prediction + kinematic-ellipse saturation), then drives `GaitScheduleUpdater` (new mode sequence) and injects the foothold into `TargetTrajectories`; reuse `SwingTrajectoryPlanner` + stance/swing constraints. Optional refinements as config: Bryson-irrecoverability weights (`w_θy=160`), Lyapunov/DCM recovery gate (LeverC), wider stance (0.18 m), land-or-bust descent.
- **Verify (gate):** `major_sagittal` (193 N) & `major_lateral` (177 N) — survival/recovery KPIs (recovery time via LeverC; `fallen` criteria), steps executed; B1/B2 (no stepping) should fall on these.
- **Risk:** largest effort; closed-loop stability; the reuse-vs-reimplement boundary for footstep logic (prefer OCS2 mechanisms + thin ViC/DCM policy on top).

---

## Metrics & logging (locked 2026-05-25)

**Principle:** one raw per-step log at sim rate (1 kHz), identical schema for both harnesses; **all KPIs derived in Python post-hoc** (reuse `heng/src` analysis/plots). C++ only logs.

**Raw columns/step:** `t`; applied disturbance (force vec, body, on/off window); decoded state (`c, ċ, θ, ω, q_j, q̇_j`); momentum `h=[p_G;k_G]`; `Ŵ`, `W_true`, `W_known`; per-foot contact wrench + point; `c_z`; DCM `ξ`; MPC solve time; mode/contact flags; guards (`slip`, `max|q̇_legs|`, `‖c−c_ref‖`). All **world-frame**, wrench `[f;τ]`, momentum `[p_G;k_G]`.

**KPIs:** `pulse_relerr` (over ON window, `regime_guard_pass` only); detection delay (first `t`: `‖Ŵ‖≥0.30‖W_true‖`, target <30 ms); bias-tracking error; peak CoM/DCM deviation; recovery time (LeverC, both axes); steps executed; MPC solve time/RTF. Targets per §07 (incl. `pulse_relerr≈15%` at `αT≥20`).

**Locked choices:** DCM `ω=√(g/c_z)` with **instantaneous** `c_z`; **GRF-only** `W_known` (no EOM); baselines B1 `SRBD↔Full`, B2 `Full` no-FF ↔ `+Ŵ`, B3 no-stepping↔stepping; **B1/B2 keep magnitudes below the stepping threshold (minor + bias); B3 uses the major (step-requiring) magnitudes**; **fall = the sim's existing reset/fall signal** (confirm its trigger when wiring B3 — don't re-implement the `c_z`/θ condition); **no C++↔Python parity test** — validate the observer via the `Ŵ` trace against the known applied force.

**Artifacts:** reuse the `heng/src` CSV schema. Harness code under `experiments/` (committed); run outputs under `experiments/results/` (**git-ignored**), organized by experiment/scenario, plus a small run-manifest (scenario params, gains, `centroidalModelType`) — keeps the working tree clean.

## Cross-cutting (impl)
- **Closed-loop (Track B):** reuse `CentroidalMpcMrtJointController` + `mujoco_sim_interface` (MRT `kp/kd=0` likely benign — verify tracking at B1).
- **Python:** plotting/analysis only; reuse `heng/src` scripts.

## Sequencing & dependencies

```
Phase 0 ─┬─► A1 ─► A2            (observer: standalone, documentable)
         └─► B1 ─► B2 ─► B3      (ViC/ADR)        B2 depends on A1 (needs Ŵ)
```
Each gate = a report-able milestone. Recommended order: **Phase 0 → A1/A2 (observer) → B1 → B2 → B3.**

## Open questions to settle during implementation (flagged per "flag uncertainty")

1. ~~Variable-inertia framing~~ — **RESOLVED (2026-05-25):** `Full`-vs-`SRBD` ablation via the `centroidalModelType` flag; no new dynamics code. See `relatorio-pi/note_ocs2.tex` and B1.
2. **`Ŵ` parameter path** through CppAD dynamics (B2) — parameter mechanism vs reference-injection fallback.
3. **MRT controller** `kp/kd=0`: **likely benign** — José runs the full centroidal model on Ubuntu with no sim issues. Verify tracking empirically at B1; isolate + unit-test fix only if it degrades the closed-loop comparison.
4. **Footstep-adaptation reuse boundary** (B3): how much OCS2 machinery vs ported Meng/DCM logic.
