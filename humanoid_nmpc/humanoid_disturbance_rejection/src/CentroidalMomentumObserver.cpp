#include "humanoid_disturbance_rejection/CentroidalMomentumObserver.h"

namespace humanoid::adr {

std::vector<double> CentroidalObserverConfig::triplePoleGains(double alpha, int order) {
  switch (order) {
    case 1:
      return {alpha};
    case 2:
      return {alpha / 3.0, alpha};
    case 3:
      return {alpha / 3.0, alpha, 3.0 * alpha};
    default:
      throw std::invalid_argument("triplePoleGains: order must be in {1,2,3}");
  }
}

double CentroidalObserverConfig::profileAlpha(ObserverGainProfile profile) {
  switch (profile) {
    case ObserverGainProfile::ShortPulse:
      return 70.0;
    case ObserverGainProfile::Agile40:
      return 40.0;
    case ObserverGainProfile::Fast:
      return 6.0;
    case ObserverGainProfile::Stable:
      return 3.5;
  }
  throw std::invalid_argument("profileAlpha: unknown profile");
}

void CentroidalObserverConfig::finalize() {
  if (order < 1 || order > kMaxObserverOrder) {
    throw std::invalid_argument("CentroidalObserverConfig: order must be in {1,2,3}");
  }
  if (gains.empty()) {
    gains = triplePoleGains(profileAlpha(profile), order);
  }
  if (static_cast<int>(gains.size()) != order) {
    throw std::invalid_argument("CentroidalObserverConfig: gains.size() must equal order");
  }
}

CentroidalMomentumObserver::CentroidalMomentumObserver(const CentroidalObserverConfig& config) : cfg_(config) {
  cfg_.finalize();
  reset();
}

void CentroidalMomentumObserver::reset() {
  hHatIntegral_.setZero();
  for (auto& g : gamma_) {
    g.setZero();
  }
  wHat_.setZero();
}

const vector6_t& CentroidalMomentumObserver::update(const vector6_t& h, const vector6_t& W_known, double dtOverride) {
  const double dt = (dtOverride > 0.0) ? dtOverride : cfg_.dt;

  // 1) predicted momentum rate:  h_hat_dot = W_known + W_hat
  const vector6_t hHatDot = W_known + wHat_;

  // 2) integrate predicted momentum (forward Euler)
  hHatIntegral_ += hHatDot * dt;

  // 3) gamma_1 = K1 (h - h_hat)   [algebraic, residual feedback]
  gamma_[0] = cfg_.gains[0] * (h - hHatIntegral_);

  // 4) integrate gamma_i, i = 2..r
  for (int i = 1; i < cfg_.order; ++i) {
    gamma_[i] += cfg_.gains[i] * (-wHat_ + gamma_[i - 1]) * dt;
  }

  // 5) new estimate:  W_hat = gamma_r
  wHat_ = gamma_[cfg_.order - 1];
  return wHat_;
}

}  // namespace humanoid::adr
