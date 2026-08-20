#pragma once

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

class DelayLockedLoop {
private:
  float Kp;
  float Ki;
  float code_integrator;

  float damp;
  float per;

  static constexpr float CARRIER_TO_CODE_RATIO = 1.0f / 1540.0f;

public:
  DelayLockedLoop(float initial_bw, float damp_val, float per_val)
      : damp(damp_val), per(per_val), code_integrator(0.0f) {
    set_bandwidth(initial_bw);
  }

  void set_bandwidth(float bw) {
    float wn = bw * 8.0f * damp / (4.0f * damp * damp + 1.0f);
    Kp = 2.0f * damp * wn;
    Ki = wn * wn * per;
  }

  float update(float I_E, float Q_E, float I_L, float Q_L,
               float carrier_doppler_hz) {
    float E_power = I_E * I_E + Q_E * Q_E;
    float L_power = I_L * I_L + Q_L * Q_L;

    float sum_power = E_power + L_power;
    float tau_err = 0.0f;

    if (sum_power > 1e-6f) {
      tau_err = (E_power - L_power) / sum_power;
    }

    code_integrator += tau_err * Ki;
    float code_filter_out = tau_err * Kp + code_integrator;

    float carrier_aiding_hz = carrier_doppler_hz * CARRIER_TO_CODE_RATIO;
    return code_filter_out + carrier_aiding_hz;
  }

  void reset() { code_integrator = 0.0f; }

  float get_integrator_state() const { return code_integrator; }
};
