#pragma once

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>

class EKF_TrackingLoop {
private:
  static constexpr float PI = 3.14159265358979323846f;

  // Векторы состояний [Фаза, Частота, Ускорение]^T
  Eigen::Vector3f X_est;
  Eigen::Vector3f X_pred;

  // Ковариационные матрицы
  Eigen::Matrix3f P_est;
  Eigen::Matrix3f P_pred;

  // Матрицы модели
  Eigen::Matrix3f Phi;
  Eigen::Matrix3f Q_noise;
  Eigen::Matrix2f R_noise;

  float T;   // Время когерентного накопления (T_coh)
  float A_k; // Текущая оценка амплитуды сигнала
  bool is_inited;

  // Вспомогательная функция для сворачивания фазы в диапазон [-PI, PI]
  void wrap_phase(float &phase) {
    phase = std::fmod(phase + PI, 2.0f * PI);
    if (phase < 0)
      phase += 2.0f * PI;
    phase -= PI;
  }

public:
  EKF_TrackingLoop(float t_coh, float initial_c_n0_dbhz = 40.0f)
      : T(t_coh), A_k(1.0f), is_inited(false) {

    // 1. Инициализация матрицы перехода состояний Phi
    Phi << 1.0f, T, 0.5f * T * T, 0.0f, 1.0f, T, 0.0f, 0.0f, 1.0f;

    // 2. Инициализация матрицы ковариации процесса Q (динамика LEO)
    // Настраивается в зависимости от ожидаемого рывка (jerk)
    float S_d = 0.1f; // Спектральная плотность ускорения
    Q_noise << (pow(T, 5) / 20.0f), (pow(T, 4) / 8.0f), (pow(T, 3) / 6.0f),
        (pow(T, 4) / 8.0f), (pow(T, 3) / 3.0f), (pow(T, 2) / 2.0f),
        (pow(T, 3) / 6.0f), (pow(T, 2) / 2.0f), T;
    Q_noise *= S_d;

    // 3. Инициализация матрицы шума измерений R
    // Зависит от C/N0 (переводим из дБ-Гц в линейный масштаб)
    float c_n0_lin = std::pow(10.0f, initial_c_n0_dbhz / 10.0f);
    float sigma_iq2 = 1.0f / (2.0f * c_n0_lin * T);
    R_noise << sigma_iq2, 0.0f, 0.0f, sigma_iq2;

    // Начальная неопределенность
    P_est = Eigen::Matrix3f::Identity() * 10.0f;
    X_est.setZero();
  }

  // Инициализация при переходе из FLL_assisted_PLL
  void init_states(float initial_freq_rad, float initial_accel_rad) {
    X_est(0) = 0.0f;              // Фазу начинаем с нуля
    X_est(1) = initial_freq_rad;  // Оценка частоты Доплера из FLL
    X_est(2) = initial_accel_rad; // Оценка ускорения из PLL
    is_inited = true;
  }

  // Главный метод обновления (твой код)
  float update(float I_p, float Q_p, float d_k) {
    if (!is_inited)
      return 0.0f;

    // Динамическая оценка амплитуды (Low-Pass Filter)
    float current_A = std::sqrt(I_p * I_p + Q_p * Q_p);
    if (A_k == 1.0f) {
      A_k = current_A;
    } else {
      A_k = 0.95f * A_k + 0.05f * current_A;
    }
    if (A_k < 1e-6f)
      A_k = 1e-6f; // Защита от деления на ноль

    // Снимаем знак навигационного бита
    float I_meas = I_p * d_k;
    float Q_meas = Q_p * d_k;
    Eigen::Vector2f Z(I_meas, Q_meas);

    // --- Этап прогноза (Prediction) ---
    X_pred = Phi * X_est;
    X_pred(0) -= X_est(1) * T;
    P_pred = Phi * P_est * Phi.transpose() + Q_noise;

    float theta_pred = X_pred(0);

    // --- Вычисление прогноза измерений и Якобиана ---
    Eigen::Vector2f h_X;
    h_X(0) = A_k * std::cos(theta_pred);
    h_X(1) = A_k * std::sin(theta_pred);

    Eigen::Matrix<float, 2, 3> H_k;
    H_k.setZero();
    H_k(0, 0) = -A_k * std::sin(theta_pred);
    H_k(1, 0) = A_k * std::cos(theta_pred);

    Eigen::Matrix2f R_scaled = R_noise * (A_k * A_k);

    // --- Вычисление усиления Калмана K_k ---
    Eigen::Matrix2f S_k = H_k * P_pred * H_k.transpose() + R_scaled;
    Eigen::Matrix<float, 3, 2> K_k = P_pred * H_k.transpose() * S_k.inverse();

    // --- Коррекция (Update) ---
    X_est = X_pred + K_k * (Z - h_X);
    P_est = (Eigen::Matrix3f::Identity() - K_k * H_k) * P_pred;

    // Ограничиваем фазу диапазоном [-PI, PI], чтобы избежать переполнения
    wrap_phase(X_est(0));

    float freq_hz = X_est(1) / (2.0f * PI);

    float phase_corr_hz = X_est(0) / (2.0f * PI * T);

    X_est(0) = 0.0f;
    // Возвращаем частоту NCO в Герцах для подстройки
    // return X_est(1) / (2.0f * PI);
    return freq_hz + phase_corr_hz;
  }

  float get_current_phase() const { return X_est(0); }
  float get_current_freq_hz() const { return X_est(1) / (2.0f * PI); }
};
