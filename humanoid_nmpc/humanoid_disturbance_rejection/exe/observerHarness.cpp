// A2: MuJoCo observer harness (balance regime).
//
// Loads the G1, holds the stance with per-joint PD, applies an xfrc_applied push
// at the torso, and runs the cascaded centroidal-momentum observer each step:
//   h        : MuJoCo mj_subtreeVel  ->  [mass*v_com ; angmom] = [linear; angular]
//   W_known  : m*g + foot-ground contact wrenches about the CoM (mj_contactForce),
//              ported from heng/src/centroidal_observer.py::compute_W_known but in
//              [f; tau] / [linear; angular] order (the observer/report convention).
//   W_true   : ground truth from the applied force  [ f ; (x_torso_ipos - c_G) x f ].
//
// Validation (this minimal A2.1 version): one long (0.7 s) push; report contact
// count, that the robot stays up, the detection latency, and the steady-window
// error of W_hat vs W_true. The full §07 sweeps + CSV logging come in A2.2.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "humanoid_disturbance_rejection/CentroidalMomentumObserver.h"

using humanoid::adr::CentroidalMomentumObserver;
using humanoid::adr::CentroidalObserverConfig;
using humanoid::adr::ObserverGainProfile;
using humanoid::adr::vector6_t;
using Vec3 = Eigen::Vector3d;

namespace {

// Default standing posture (from g1_centroidal_mpc reference.info; same as mujocoSimPDStand).
const std::unordered_map<std::string, double>& posture() {
  static const std::unordered_map<std::string, double> q = {
      {"left_hip_pitch_joint", -0.05},  {"left_knee_joint", 0.1},   {"left_ankle_pitch_joint", -0.05},
      {"right_hip_pitch_joint", -0.05}, {"right_knee_joint", 0.1},  {"right_ankle_pitch_joint", -0.05},
  };
  return q;
}

constexpr double kBaseHeight = 0.7925;
constexpr double kKp = 1500.0;
constexpr double kKd = 2.0;

}  // namespace

int main(int argc, char** argv) {
  const std::string scene =
      (argc > 1) ? argv[1] : "robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml";

  char err[1024] = "";
  mjModel* m = mj_loadXML(scene.c_str(), nullptr, err, sizeof(err));
  if (!m) {
    std::cerr << "mj_loadXML failed: " << err << std::endl;
    return 1;
  }
  m->opt.timestep = 0.001;  // 1 kHz, matching the report
  mjData* d = mj_makeData(m);

  const double mass = mj_getTotalmass(m);
  const Vec3 gravity(m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]);

  const int floor_id = mj_name2id(m, mjOBJ_GEOM, "floor");
  const int torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  const int lankle = mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link");
  const int rankle = mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link");
  if (floor_id < 0 || torso_id < 0 || lankle < 0 || rankle < 0) {
    std::cerr << "missing body/geom (floor/torso/ankle) in model" << std::endl;
    return 1;
  }

  // Standing initial state.
  mj_resetData(m, d);
  d->qpos[2] = kBaseHeight;
  d->qpos[3] = 1.0;  // base quat (w,x,y,z) = identity
  d->qpos[4] = d->qpos[5] = d->qpos[6] = 0.0;
  for (const auto& [name, ang] : posture()) {
    const int j = mj_name2id(m, mjOBJ_JOINT, name.c_str());
    if (j >= 0) d->qpos[m->jnt_qposadr[j]] = ang;
  }
  mj_forward(m, d);

  // Per-actuator joint indices + PD setpoint (hold the initial posture).
  std::vector<int> qadr(m->nu), vadr(m->nu);
  std::vector<double> qdes(m->nu);
  for (int a = 0; a < m->nu; ++a) {
    const int j = m->actuator_trnid[2 * a];
    qadr[a] = m->jnt_qposadr[j];
    vadr[a] = m->jnt_dofadr[j];
    qdes[a] = d->qpos[qadr[a]];
  }

  CentroidalObserverConfig cfg;
  cfg.order = 3;
  cfg.dt = m->opt.timestep;
  cfg.profile = ObserverGainProfile::ShortPulse;  // alpha = 70
  CentroidalMomentumObserver obs(cfg);

  // Push schedule: +x at the torso, after a settle. Magnitude is arg 2 (default
  // 30 N — in-regime for the fixed-posture PD; larger pushes topple it, which is
  // the motivation for the ViC/stepping recovery on the B-track).
  const double settle = 0.5, push_dur = 0.7;
  const double push_fx = (argc > 2) ? std::stod(argv[2]) : 30.0;
  const double t0 = settle, t1 = settle + push_dur;
  const Vec3 push_force(push_fx, 0.0, 0.0);
  const double sim_T = settle + push_dur + 0.3;
  const int N = static_cast<int>(sim_T / m->opt.timestep);

  int max_ncon = 0, detect_step = -1;
  double err_sum = 0.0;
  int err_cnt = 0;
  vector6_t What_sample = vector6_t::Zero(), Wtrue_sample = vector6_t::Zero();

  for (int k = 0; k < N; ++k) {
    const double t = k * m->opt.timestep;
    const bool pushing = (t >= t0 && t < t1);

    // Control + disturbance (set before step1, which computes actuation).
    for (int a = 0; a < m->nu; ++a) {
      d->ctrl[a] = kKp * (qdes[a] - d->qpos[qadr[a]]) + kKd * (-d->qvel[vadr[a]]);
    }
    mju_zero(d->xfrc_applied, 6 * m->nbody);
    if (pushing) {
      d->xfrc_applied[6 * torso_id + 0] = push_force.x();
      d->xfrc_applied[6 * torso_id + 1] = push_force.y();
      d->xfrc_applied[6 * torso_id + 2] = push_force.z();
    }

    mj_step(m, d);           // full step: forward (solves contact forces) + integrate
    mj_subtreeVel(m, d);     // subtree momentum, consistent with this step's solved contacts

    const Vec3 vcom(d->subtree_linvel[0], d->subtree_linvel[1], d->subtree_linvel[2]);
    const Vec3 angmom(d->subtree_angmom[0], d->subtree_angmom[1], d->subtree_angmom[2]);
    const Vec3 cG(d->subtree_com[0], d->subtree_com[1], d->subtree_com[2]);

    vector6_t h;
    h.head<3>() = mass * vcom;
    h.tail<3>() = angmom;

    // W_known = m*g + sum of foot-ground contact wrenches about the CoM.
    vector6_t W_known = vector6_t::Zero();
    W_known.head<3>() = mass * gravity;
    max_ncon = std::max(max_ncon, d->ncon);
    for (int c = 0; c < d->ncon; ++c) {
      const mjContact& con = d->contact[c];
      const int gA = con.geom[0], gB = con.geom[1];
      const int bA = m->geom_bodyid[gA], bB = m->geom_bodyid[gB];
      const bool footA = (bA == lankle || bA == rankle);
      const bool footB = (bB == lankle || bB == rankle);
      const bool floorA = (gA == floor_id), floorB = (gB == floor_id);
      if (!((footA && floorB) || (footB && floorA))) continue;

      mjtNum f6[6];
      mj_contactForce(m, d, c, f6);
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(con.frame);
      const double sign = footB ? 1.0 : -1.0;  // force on the foot body
      const Vec3 f_world = sign * (R.transpose() * Eigen::Map<const Vec3>(f6));
      const Vec3 tau_world = sign * (R.transpose() * Eigen::Map<const Vec3>(f6 + 3));
      const Vec3 x(con.pos[0], con.pos[1], con.pos[2]);
      W_known.head<3>() += f_world;
      W_known.tail<3>() += tau_world + (x - cG).cross(f_world);
    }

    const vector6_t& What = obs.update(h, W_known);

    // Ground truth during the push.
    vector6_t Wtrue = vector6_t::Zero();
    if (pushing) {
      const Vec3 xip(d->xipos[3 * torso_id], d->xipos[3 * torso_id + 1], d->xipos[3 * torso_id + 2]);
      Wtrue.head<3>() = push_force;
      Wtrue.tail<3>() = (xip - cG).cross(push_force);
      const double wt = Wtrue.norm();
      if (detect_step < 0 && What.norm() >= 0.30 * wt) detect_step = k;
      if (t >= t0 + 0.5 * push_dur) {  // steady window: last half of the pulse
        err_sum += (What - Wtrue).norm() / wt;
        ++err_cnt;
        What_sample = What;
        Wtrue_sample = Wtrue;
      }
    }
  }

  const double detect_ms = (detect_step >= 0) ? ((detect_step + 1) * m->opt.timestep - t0) * 1e3 : -1.0;
  const double steady_relerr = (err_cnt > 0) ? err_sum / err_cnt : -1.0;
  const double final_z = d->qpos[2];

  std::cout << "[observerHarness] mass=" << mass << " kg  dt=" << m->opt.timestep << " s\n";
  std::cout << "  max contacts (ncon) = " << max_ncon << "  final base z = " << final_z << " m\n";
  std::cout << "  push: +x " << push_fx << " N at torso for " << push_dur << " s\n";
  std::cout << "  W_true (steady) = " << Wtrue_sample.transpose() << "\n";
  std::cout << "  W_hat  (steady) = " << What_sample.transpose() << "\n";
  std::cout << "  detection delay   = " << detect_ms << " ms\n";
  std::cout << "  steady rel error  = " << steady_relerr * 100.0 << " %\n";

  const bool stood = final_z > 0.5;
  const bool contacts_ok = max_ncon > 0;
  const bool detect_ok = detect_ms > 0.0 && detect_ms < 50.0;
  const bool track_ok = steady_relerr >= 0.0 && steady_relerr < 0.30;  // lenient: model mismatch expected
  std::cout << "[observerHarness] stood=" << stood << " contacts=" << contacts_ok
            << " detect_ok=" << detect_ok << " track_ok=" << track_ok << "\n";

  mj_deleteData(d);
  mj_deleteModel(m);

  if (stood && contacts_ok && detect_ok && track_ok) {
    std::cout << "[observerHarness] PASS\n";
    return 0;
  }
  std::cerr << "[observerHarness] FAIL\n";
  return 2;
}
