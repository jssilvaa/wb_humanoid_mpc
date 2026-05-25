// Sanity check for the Pinocchio centroidal-momentum helper (no MuJoCo).
// Loads the G1 URDF and verifies two physical invariants:
//   (1) h(q, v=0) = 0                  -- zero velocity => zero momentum
//   (2) pure base x-translation v_x    -- total linear momentum p_x = mass * v_x
//       (with all joints fixed, the whole body translates with the base)
// This also confirms the [linear; angular] ordering of h.

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Core>

#include "humanoid_disturbance_rejection/CentroidalMomentumCalculator.h"

int main(int argc, char** argv) {
  const std::string urdf =
      (argc > 1) ? argv[1] : "robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf";

  humanoid::adr::CentroidalMomentumCalculator calc(urdf);

  const double mass = calc.totalMass();
  const Eigen::VectorXd q = calc.neutralConfiguration();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(calc.nv());

  const auto m0 = calc.momentum(q, v);
  std::cout << "[centroidalSanityCheck] nq=" << calc.nq() << " nv=" << calc.nv()
            << " mass=" << mass << " kg\n";
  std::cout << "  h(v=0)    = " << m0.h.transpose() << "   (expect ~0)\n";
  std::cout << "  com       = " << m0.com.transpose() << "\n";

  // Pure base x-translation at 0.5 m/s (free-flyer v[0] = local linear x; identity
  // base orientation at neutral => world x).
  const double vx = 0.5;
  v(0) = vx;
  const auto m1 = calc.momentum(q, v);
  const double px = m1.h(0);
  std::cout << "  h(vx=0.5) = " << m1.h.transpose() << "\n";
  std::cout << "  p_x=" << px << "   (expect mass*vx=" << mass * vx << ")\n";

  const bool zero_ok = m0.h.norm() < 1e-9;
  const bool px_ok = std::abs(px - mass * vx) < 1e-6 * std::max(1.0, mass);
  if (zero_ok && px_ok) {
    std::cout << "[centroidalSanityCheck] PASS\n";
    return 0;
  }
  std::cerr << "[centroidalSanityCheck] FAIL (zero_ok=" << zero_ok << ", px_ok=" << px_ok << ")\n";
  return 2;
}
