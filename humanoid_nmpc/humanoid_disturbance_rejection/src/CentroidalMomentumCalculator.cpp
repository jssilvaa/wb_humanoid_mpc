#include "humanoid_disturbance_rejection/CentroidalMomentumCalculator.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace humanoid::adr {

pinocchio::Model CentroidalMomentumCalculator::buildModel(const std::string& urdfPath) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), model);
  return model;
}

CentroidalMomentumCalculator::CentroidalMomentumCalculator(const std::string& urdfPath)
    : model_(buildModel(urdfPath)), data_(model_) {}

CentroidalMomentumCalculator::Result CentroidalMomentumCalculator::momentum(const Eigen::VectorXd& q,
                                                                            const Eigen::VectorXd& v) {
  // Fills data_.hg (centroidal momentum, a spatial Force) and data_.com[0] (CoM).
  pinocchio::computeCentroidalMomentum(model_, data_, q, v);
  Result out;
  out.h.head<3>() = data_.hg.linear();   // p_G (linear momentum)
  out.h.tail<3>() = data_.hg.angular();  // k_G (angular momentum)
  out.com = data_.com[0];
  return out;
}

int CentroidalMomentumCalculator::nq() const { return model_.nq; }

int CentroidalMomentumCalculator::nv() const { return model_.nv; }

double CentroidalMomentumCalculator::totalMass() const { return pinocchio::computeTotalMass(model_); }

Eigen::VectorXd CentroidalMomentumCalculator::neutralConfiguration() const { return pinocchio::neutral(model_); }

}  // namespace humanoid::adr
