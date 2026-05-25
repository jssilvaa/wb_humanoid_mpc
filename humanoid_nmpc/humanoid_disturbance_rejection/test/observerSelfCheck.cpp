// Self-contained convergence check for the cascaded centroidal-momentum observer.
// No MuJoCo / Pinocchio: a synthetic "plant" integrates a constant step external
// wrench W_bar with forward Euler; the observer is fed (h, W_known=0) each step and
// must converge W_hat -> W_bar. We also measure the detection delay (first time
// ||W_hat|| >= 0.30 ||W_bar||), which for a triple pole at -alpha should be
// t30 = 1.9138 / alpha  (=> ~27.3 ms at alpha = 70).

// measure was added by me to test t90, just to get some idea on the time interval after which the measurement is usable with the most precision (should still be useful before as ff)

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "humanoid_disturbance_rejection/CentroidalMomentumObserver.h"

using humanoid::adr::CentroidalMomentumObserver;
using humanoid::adr::CentroidalObserverConfig;
using humanoid::adr::ObserverGainProfile;
using humanoid::adr::vector6_t;

int main() {
  CentroidalObserverConfig cfg;
  cfg.order = 3;
  cfg.dt = 0.001;
  cfg.profile = ObserverGainProfile::ShortPulse;  // alpha = 70
  CentroidalMomentumObserver obs(cfg);

  vector6_t W_bar;
  W_bar << 12.0, -8.0, 5.0, 1.0, -2.0, 3.0;  // [f(3); tau(3)]
  const double W_bar_norm = W_bar.norm();
  const double alpha = 70.0;

  const int n_steps = 2000;  // 2 s at 1 kHz (alpha*T = 140 >> 1)
  vector6_t h = vector6_t::Zero();          // true centroidal momentum
  const vector6_t W_known = vector6_t::Zero();

  int detect_step = -1;
  int measure_step = -1;
  for (int k = 0; k < n_steps; ++k) {
    // True plant: h_dot = W_known + W_bar  (forward Euler).
    h += (W_known + W_bar) * cfg.dt;
    const vector6_t& W_hat = obs.update(h, W_known);
    if (detect_step < 0 && W_hat.norm() >= 0.30 * W_bar_norm) {
      detect_step = k;
    }
    else if (detect_step > 0 && measure_step < 0 && W_hat.norm() >= 0.90 * W_bar_norm) {
      measure_step = k;
    }
  }

  const vector6_t W_hat = obs.wHat();
  const double rel_err = (W_hat - W_bar).norm() / W_bar_norm;
  const double detect_ms = (detect_step >= 0) ? (detect_step + 1) * cfg.dt * 1e3 : -1.0;
  const double measured_ms = (measure_step >= 0) ? (measure_step + 1) * cfg.dt * 1e3 : -1.0;
  const double t30_expected_ms = 1.9138 / alpha * 1e3;
  // t90: solve y3(x) = 1 - e^{-x}(1 + x + x^2/2) = 0.90  =>  e^{-x}(1+x+x^2/2) = 0.10
  //   =>  x90 ~ 5.322 (numeric)  =>  t90 = x90 / alpha  (~76 ms at alpha=70).
  const double x90 = 5.322;
  const double t90_expected_ms = x90 / alpha * 1e3;

  std::cout << "[observerSelfCheck] profile=ShortPulse(alpha=70) order=3 dt=1ms\n";
  std::cout << "  W_bar  = " << W_bar.transpose() << "\n";
  std::cout << "  W_hat  = " << W_hat.transpose() << "\n";
  std::cout << "  final relative error = " << rel_err * 100.0 << " %\n";
  std::cout << "  detection delay        = " << detect_ms << " ms (expected t30 ~ " << t30_expected_ms << " ms)\n";
  std::cout << "  measurement delay      = " << measured_ms << " ms (expected t90 ~ " << t90_expected_ms << " ms)\n";

  const bool converged = rel_err < 1e-2;                        // < 1 % after 2 s
  const bool detect_ok = detect_ms > 15.0 && detect_ms < 40.0;  // around 27 ms
  const bool measure_ok = measured_ms < 100.0;                  // lt 100ms
  if (converged && detect_ok && measure_ok) {
    std::cout << "[observerSelfCheck] PASS\n";
    return 0;
  }
  std::cerr << "[observerSelfCheck] FAIL (converged=" << converged << ", detect_ok=" << detect_ok << ", measure_ok=" << measure_ok << ")\n";
  return 2;
}
