// Pinocchio-based centroidal-momentum helper for the wrench observer.
//
// Builds a Pinocchio model with a free-flyer base from a URDF and computes the
// 6D centroidal momentum h and the CoM from generalized coords/vels (q, v) via
// the composite-rigid-body / centroidal algorithms (the C++ analog of the Python
// prototype's compute_cmm). Output ordering matches the observer and the report:
//   h = [ p_G ; k_G ] = [ linear ; angular ]   (Pinocchio's Force order),
// expressed at the CoM in the world-aligned (centroidal) frame.

#pragma once

#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "humanoid_disturbance_rejection/CentroidalMomentumObserver.h"  // vector6_t

namespace humanoid::adr {

class CentroidalMomentumCalculator {
 public:
  struct Result {
    vector6_t h;          // centroidal momentum [linear(3); angular(3)]
    Eigen::Vector3d com;  // CoM position in the world frame
  };

  explicit CentroidalMomentumCalculator(const std::string& urdfPath);

  // q : Pinocchio position (free-flyer: [x,y,z, qx,qy,qz,qw, joints]), size nq()
  // v : Pinocchio velocity (free-flyer: [v_lin, v_ang (local frame), joint_vel]), size nv()
  Result momentum(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  int nq() const;
  int nv() const;
  double totalMass() const;
  Eigen::VectorXd neutralConfiguration() const;

 private:
  static pinocchio::Model buildModel(const std::string& urdfPath);

  pinocchio::Model model_;
  pinocchio::Data data_;
};

}  // namespace humanoid::adr
