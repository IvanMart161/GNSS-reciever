#pragma once

#define _USE_MATH_DEFINES
#include "dll_loop.hpp" // [NEW] Подключаем математику DLL
#include "gnss_ekf.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

static constexpr float PI = 3.14159265358979323846f;

// [NEW] Структура ответа контроллера, содержит обе поправки
struct TrackingResponse {
  int32_t carrier_nco_adjust;
  int32_t code_nco_adjust;
  float fine_tune_code_hz; // Для вывода графиков в Python
};

// ==============================================================================
// 1. Модуль Frequency Pulling (Грубые 20 мс)
// ==============================================================================
class FrequencyPuller {
private:
  float T;
  size_t N_target;
  std::vector<float> freq_errors;
  float I_prev, Q_prev;

public:
  FrequencyPuller(float integration_time, size_t size)
      : T(integration_time), N_target(size), I_prev(0.0f), Q_prev(0.0f) {
    freq_errors.reserve(size);
  }

  float process_ms(float I_p, float Q_p, bool &is_finished) {
    if (freq_errors.empty() && I_prev == 0.0f && Q_prev == 0.0f) {
      I_prev = I_p;
      Q_prev = Q_p;
      is_finished = false;
      return 0.0f;
    }

    float cross = I_prev * Q_p - Q_prev * I_p;
    float dot = I_prev * I_p + Q_prev * Q_p;
    float delta_phi = std::atan2(cross, dot);
    float delta_f = delta_phi / (2.0f * PI * T);

    freq_errors.push_back(delta_f);
    I_prev = I_p;
    Q_prev = Q_p;

    is_finished = (freq_errors.size() >= N_target);

    return is_finished ? get_current_estimate() : 0.0f;
  }

  float get_current_estimate() const {
    if (freq_errors.empty())
      return 0.0f;
    size_t drop_count = static_cast<size_t>(freq_errors.size() * 0.15f);
    if (freq_errors.size() <= drop_count * 2 + 1) {
      float sum = 0.0f;
      for (float f : freq_errors)
        sum += f;
      return sum / static_cast<float>(freq_errors.size());
    }
    std::vector<float> sorted_errors = freq_errors;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    float sum = 0.0f;
    for (size_t i = drop_count; i < sorted_errors.size() - drop_count; ++i) {
      sum += sorted_errors[i];
    }
    return sum / static_cast<float>(sorted_errors.size() - 2 * drop_count);
  }
};

// ==============================================================================
// 2. FLL-ASSISTED-PLL. FLL 2nd order & PLL 3rd order
// ==============================================================================
class FLL_assisted_PLL {
private:
  static constexpr float PI = 3.14159265358979323846f;
  float T, Kp_f, Ki_f, Kp_p, Ki_p, Ki2_p;
  float I_freq, I_accel, I_prev, Q_prev;
  bool is_first_sample;

public:
  FLL_assisted_PLL(float bw_fll, float bw_pll, float integration_time)
      : T(integration_time), I_freq(0.0f), I_accel(0.0f), I_prev(0.0f),
        Q_prev(0.0f), is_first_sample(true) {

    Kp_f = 4.0f * bw_fll * T;
    Ki_f = Kp_f * Kp_f;

    float omega_n_p = bw_pll / 0.7845f;
    Kp_p = 2.4f * omega_n_p;
    Ki_p = 1.1f * omega_n_p * omega_n_p;
    Ki2_p = omega_n_p * omega_n_p * omega_n_p;
  }

  void init_frequency(float initial_freq_hz) {
    I_freq = initial_freq_hz * 2.0f * PI;
    I_accel = 0.0f;
    is_first_sample = true;
  }

  float update(float I_p, float Q_p, bool pll_enabled = true) {
    if (is_first_sample) {
      I_prev = I_p;
      Q_prev = Q_p;
      is_first_sample = false;
      return I_freq / (2.0f * PI);
    }

    float cross = I_prev * Q_p - Q_prev * I_p;
    float dot = I_prev * I_p + Q_prev * Q_p;
    float sign_dot = (dot >= 0.0f) ? 1.0f : -1.0f;
    float norm = std::sqrt(dot * dot + cross * cross);

    float delta_theta = (norm > 1e-6f) ? (cross * sign_dot) / norm : 0.0f;
    float err_omega = delta_theta / T;

    I_prev = I_p;
    Q_prev = Q_p;

    float delta_phi = 0.0f;
    if (pll_enabled) {
      if (std::abs(I_p) > 1e-6f) {
        delta_phi = std::atan(Q_p / I_p);
      } else {
        delta_phi = (Q_p > 0.0f) ? (PI / 2.0f) : (-PI / 2.0f);
      }
    }

    if (pll_enabled)
      I_accel += Ki2_p * delta_phi * T;
    I_freq += (Ki_p * delta_phi + Ki_f * err_omega + I_accel) * T;

    float total_corr_rad = Kp_p * delta_phi + Kp_f * err_omega + I_freq;
    return total_corr_rad / (2.0f * PI);
  }
};
// ==============================================================================
// 3. FSM КОНТРОЛЛЕР КАНАЛА (Adaptive State Machine)
// ==============================================================================
enum ChannelState {
  STATE_PULLING = 0,
  STATE_FLL_ONLY = 1,
  STATE_FLL_ASSISTED_PLL = 2,
  STATE_EKF_TRACKING = 3
};

class TrackingChannelController {
private:
  FrequencyPuller puller;
  FLL_assisted_PLL tracking_loop;
  EKF_TrackingLoop ekf;
  DelayLockedLoop dll;
  ChannelState state;

  double hz_to_nco_scale;
  float current_fine_tune_hz;
  float current_code_tune_hz;

  uint32_t fll_only_samples;
  uint32_t pll_samples;

  // [NEW] Индикаторы захвата (Lock Detectors)
  float prev_freq_hz;
  uint32_t fll_lock_count;
  float smoothed_pli;

  float base_doppler_hz;
  float sum_I, sum_Q;
  int ekf_ms_count;

public:
  TrackingChannelController(float f_sys_hz, float bw_fll = 10.0f,
                            float bw_pll = 15.0f, float T = 0.001f)
      : puller(T, 20), tracking_loop(bw_fll, bw_pll, T), ekf(0.020f, 40.0f),
        dll(2.0f, 0.707f, T), state(STATE_PULLING), current_fine_tune_hz(0.0f),
        current_code_tune_hz(0.0f), fll_only_samples(0), pll_samples(0),
        prev_freq_hz(0.0f), // [NEW] Инициализация переменных детекторов
        fll_lock_count(0),  // [NEW]
        smoothed_pli(0.0f), // [NEW]
        base_doppler_hz(0.0f), sum_I(0.0f), sum_Q(0.0f), ekf_ms_count(0) {
    hz_to_nco_scale = 4294967296.0 / static_cast<double>(f_sys_hz);
  }

  TrackingResponse process_sample(int32_t raw_i_e, int32_t raw_q_e,
                                  int32_t raw_i_p, int32_t raw_q_p,
                                  int32_t raw_i_l, int32_t raw_q_l) {

    // Приведение всех корреляторов к float с инверсией Q-канала
    float I_e = static_cast<float>(raw_i_e);
    float Q_e = -static_cast<float>(raw_q_e);
    float I_p = static_cast<float>(raw_i_p);
    float Q_p = -static_cast<float>(raw_q_p);
    float I_l = static_cast<float>(raw_i_l);
    float Q_l = -static_cast<float>(raw_q_l);

    // --- ВЕТКА НЕСУЩЕЙ (Адаптивный FSM) ---
    switch (state) {

    // 1. Грубая оценка частоты (Блочный фильтр, строго 20 мс)
    case STATE_PULLING: {
      bool pulling_done = false;
      current_fine_tune_hz = puller.process_ms(I_p, Q_p, pulling_done);
      if (pulling_done) {
        tracking_loop.init_frequency(current_fine_tune_hz);
        state = STATE_FLL_ONLY;

        // Подготавливаем переменные для следующего шага
        prev_freq_hz = current_fine_tune_hz;
        fll_only_samples = 0;
        fll_lock_count = 0;
      }
      break;
    }

    // 2. Удержание частоты (Ожидание стабилизации FLI)
    case STATE_FLL_ONLY: {
      float new_freq_hz = tracking_loop.update(I_p, Q_p, false);

      // Вычисляем дельту (скачок) частоты
      float freq_step = std::abs(new_freq_hz - prev_freq_hz);
      prev_freq_hz = new_freq_hz;
      current_fine_tune_hz = new_freq_hz;

      fll_only_samples++;

      // Детектор захвата: частота не должна прыгать более чем на 1.5 Гц
      if (freq_step < 1.5f) {
        fll_lock_count++;
      } else {
        fll_lock_count = 0; // При сильном рывке сбрасываем уверенность
      }

      // Умный переход: Ждем минимум 20 мс для разгона, требуем 15 мс идеальной
      // стабильности
      if (fll_only_samples >= 20 && fll_lock_count >= 15) {
        state = STATE_FLL_ASSISTED_PLL;

        // Подготавливаем переменные для следующего шага
        pll_samples = 0;
        smoothed_pli = 0.0f;
      }
      break;
    }

    // 3. Удержание фазы (Ожидание стягивания энергии в канал I через PLI)
    case STATE_FLL_ASSISTED_PLL: {
      current_fine_tune_hz = tracking_loop.update(I_p, Q_p, true);

      // Мгновенный индикатор захвата фазы (PLI)
      float I_sq = I_p * I_p;
      float Q_sq = Q_p * Q_p;
      float current_pli = (I_sq - Q_sq) / (I_sq + Q_sq + 1e-6f);

      // Сглаживание через Low-Pass Filter
      smoothed_pli = 0.95f * smoothed_pli + 0.05f * current_pli;

      pll_samples++;

      // Умный переход: Ждем минимум 50 мс для накопления уверенности.
      // Включаем EKF только тогда, когда фаза идеально выровнялась (PLI > 0.8)
      if (pll_samples >= 50 && smoothed_pli > 0.8f) {
        ekf.init_states(current_fine_tune_hz * 2.0f * PI, 0.0f);
        state = STATE_EKF_TRACKING;

        // Подготавливаем переменные для РФК и сужаем кодовую петлю
        sum_I = 0.0f;
        sum_Q = 0.0f;
        ekf_ms_count = 0;
        dll.set_bandwidth(0.5f);
      }
      break;
    }

    // 4. Глубокое слежение через Расширенный Фильтр Калмана
    case STATE_EKF_TRACKING: {
      sum_I += I_p;
      sum_Q += Q_p;
      ekf_ms_count++;

      if (ekf_ms_count >= 20) {
        float d_k = (sum_I >= 0.0f) ? 1.0f : -1.0f;
        current_fine_tune_hz = ekf.update(sum_I, sum_Q, d_k);
        sum_I = 0.0f;
        sum_Q = 0.0f;
        ekf_ms_count = 0;
      }
      break;
    }
    }

    // --- ВЕТКА КОДА (DLL работает параллельно и всегда) ---
    float total_carrier_doppler = base_doppler_hz + current_fine_tune_hz;
    current_code_tune_hz =
        dll.update(I_e, Q_e, I_l, Q_l, total_carrier_doppler);

    // --- ФОРМИРОВАНИЕ ОТВЕТА ---
    TrackingResponse response;
    response.carrier_nco_adjust = static_cast<int32_t>(
        std::round(current_fine_tune_hz * hz_to_nco_scale));
    response.code_nco_adjust = static_cast<int32_t>(
        std::round(current_code_tune_hz * hz_to_nco_scale));
    response.fine_tune_code_hz = current_code_tune_hz;

    return response;
  }

  ChannelState get_state() const { return state; }
  float get_fine_tune_hz() const { return current_fine_tune_hz; }
};
