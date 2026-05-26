// Cascaded centroidal-momentum observer for external-wrench estimation.
//
// C++ port of control-rejection/heng/src/centroidal_observer.py (Heng et al.;
// cascaded centroidal momentum-based disturbance observer on non-compensated
// dynamics). Pure Eigen — no MuJoCo / Pinocchio / OCS2 dependency. The plant-side
// inputs (centroidal momentum h, and the known wrench W_known = gravity + measured
// contact wrenches about the CoM) are supplied by the caller.
//
// CONVENTION (see relatorio-pi/note_ocs2.tex): all 6-vectors are ordered
//   [ f ; tau ]  ==  [ linear ; angular ],
// matching OCS2's centroidal momentum order [p_G ; k_G] and the report. This is
// the OPPOSITE of the Python prototype's internal [tau ; f] / [angular ; linear]
// order — we deliberately use the report/OCS2 order to avoid the Pi permutation.
//
// Estimator (predictor):           h_hat_dot = W_known + W_hat
// Cascade (order r), forward Euler: gamma_1 = K1 (h - h_hat)
//                                   gamma_i = Ki * integral(-W_hat + gamma_{i-1}) dt,  i = 2..r
//                                   W_hat   = gamma_r
// Triple-pole design at -alpha per channel: K1 = alpha/3, K2 = alpha, K3 = 3*alpha.

#pragma once

#include <array>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

namespace humanoid::adr {

using vector6_t = Eigen::Matrix<double, 6, 1>;

inline constexpr int kMaxObserverOrder = 3; // max observer order

// Gain profiles (triple-pole design at -alpha)
enum class ObserverGainProfile {
  ShortPulse,  // alpha = 70   -> K = [23.3, 70, 210], tau = 14 ms      (centroidal default)
  Agile40,     // alpha = 40   -> K = [13.3, 40, 120], tau = 25 ms
  Fast,        // alpha = 6    -> K = [2.0,  6,  18],  tau = 167 ms
  Stable,      // alpha = 3.5  -> K = [1.17, 3.5, 10.5], tau = 286 ms
};

struct CentroidalObserverConfig {
  int order = 3;            // cascade order r, in {1, .., kMaxObserverOrder}
  double dt = 0.001;        // control timestep [s]
  ObserverGainProfile profile = ObserverGainProfile::ShortPulse;
  std::vector<double> gains;  // [K1, .., Kr]; if empty, filled from `profile` by finalize()

  // Triple-pole gains [K1..Kr] for a pole at -alpha: order1=[a], order2=[a/3, a], order3=[a/3, a, 3a].
  static std::vector<double> triplePoleGains(double alpha, int order);

  // alpha associated with a profile.
  static double profileAlpha(ObserverGainProfile profile);

  // Fills `gains` from `profile` when empty, then validates order/size. Throws on misconfig.
  void finalize();
};

class CentroidalMomentumObserver {
 public:
  explicit CentroidalMomentumObserver(const CentroidalObserverConfig& config);

  // Advance one timestep and return the updated estimate W_hat = [f_ext ; tau_ext].
  //   h          : measured centroidal momentum   [p_G ; k_G] = [linear ; angular]
  //   W_known    : gravity + measured contact wrenches about the CoM, same ordering
  //   dtOverride : if > 0, use this timestep for this step's forward-Euler integration (for
  //                variable-rate control loops); otherwise the configured cfg.dt is used.
  const vector6_t& update(const vector6_t& h, const vector6_t& W_known, double dtOverride = -1.0);

  const vector6_t& wHat() const { return wHat_; }
  void reset();

  int order() const { return cfg_.order; }
  double dt() const { return cfg_.dt; }
  const std::vector<double>& gains() const { return cfg_.gains; }

 private:
  CentroidalObserverConfig cfg_;
  vector6_t hHatIntegral_;                          // h_hat = integral of (W_known + W_hat)
  std::array<vector6_t, kMaxObserverOrder> gamma_;  // gamma_1 .. gamma_r (first `order` used)
  vector6_t wHat_;
};

}  // namespace humanoid::adr
