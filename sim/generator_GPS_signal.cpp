#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

constexpr double fs = 38.192e6;     // Частота дискретизации АЦП (38.192 МГц)
constexpr double T_sim = 8.0;       // Длительность (8 c)
constexpr double f_if = 9.548e6;    // Промежуточная частота (9.548 МГц)
constexpr double f_L1 = 1575.42e6;  // Частота несущей GPS L1 (Гц)
constexpr double f_doppler = 521.0; // Начальная частота Доплера (Гц)
constexpr double f_dot = -80;     // Ускорение Доплера (Гц/с)
constexpr double f_chip = 1.023e6;  // Скорость C/A кода (Гц)
constexpr double SNR_db = -20.0;    // ОСШ (дБ)
constexpr size_t FILTER_ORDER = 8;  // Порядок БИХ фильтра
constexpr double code_shift_chips = 667;

// =========================================================================
// 2. КЛАСС ЦИФРОВОГО ФИЛЬТРА (ПАВ)
// =========================================================================

class IIRFilter {
private:
  std::array<double, FILTER_ORDER + 1> b;
  std::array<double, FILTER_ORDER + 1> a;
  std::array<double, FILTER_ORDER> w; // Линия задержки

public:
  IIRFilter(const std::array<double, FILTER_ORDER + 1> &b_coeffs,
            const std::array<double, FILTER_ORDER + 1> &a_coeffs)
      : b(b_coeffs), a(a_coeffs) {
    w.fill(0.0);
  }

  inline double process(double x) {
    double y = b[0] * x + w[0];

    for (int i = 0; i < FILTER_ORDER - 1; ++i) {
      w[i] = b[i + 1] * x + w[i + 1] - a[i + 1] * y;
    }

    w[FILTER_ORDER - 1] = b[FILTER_ORDER] * x - a[FILTER_ORDER] * y;

    return y;
  }
};

std::array<int, 1023> generate_prn_code() {
  std::array<int, 1023> prn_bpsk;
  std::array<int, 10> G1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  std::array<int, 10> G2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  for (int i = 0; i < 1023; ++i) {
    int g1_out = G1[9];
    int g2_out = G2[1] ^ G2[5];
    int prn_bit = g1_out ^ g2_out;

    prn_bpsk[i] = 1 - 2 * prn_bit; // 0 -> +1, 1 -> -1

    int g1_fb = G1[2] ^ G1[9];
    int g2_fb = G2[1] ^ G2[2] ^ G2[5] ^ G2[7] ^ G2[8] ^ G2[9];

    for (int j = 9; j > 0; --j) {
      G1[j] = G1[j - 1];
      G2[j] = G2[j - 1];
    }
    G1[0] = g1_fb;
    G2[0] = g2_fb;
  }
  return prn_bpsk;
}

std::vector<int> generate_nav_message() {
  std::vector<int> full_bin_msg;
  full_bin_msg.reserve(400);

  // Мусорные биты
  for (int i = 0; i < 50; ++i) {
    full_bin_msg.push_back(1);
    full_bin_msg.push_back(0);
  }

  auto append_30bits = [&full_bin_msg](uint32_t val) {
    for (int i = 29; i >= 0; --i) {
      full_bin_msg.push_back((val >> i) & 1);
    }
  };

  uint32_t word1_val = 139 << 22;
  append_30bits(word1_val);

  uint32_t tow_val = 100000;
  uint32_t subframe_id = 2;
  uint32_t word2_val = (tow_val << 13) | (subframe_id << 8);
  append_30bits(word2_val);

  const std::array<uint32_t, 8> payload_words = {
      0x000000C0, // Слово 3
      0x00000100, // Слово 4
      0x00000140, // Слово 5
      0x00000180, // Слово 6
      0x000001C0, // Слово 7
      0x00000200, // Слово 8
      0x00000240, // Слово 9
      0x00000280  // Слово 10
  };

  for (uint32_t val : payload_words) {
    append_30bits(val);
  }

  return full_bin_msg;
}

int main() {
  std::cout << "Начало генерации GPS сигнала (C++)..." << std::endl;

  auto prn_bpsk = generate_prn_code();
  auto nav_msg = generate_nav_message();

  std::vector<int> nav_bpsk(nav_msg.size());
  for (size_t i = 0; i < nav_msg.size(); ++i) {
    nav_bpsk[i] = 1 - 2 * nav_msg[i];
  }

  double signal_power = 0.5;
  double SNR_linear = std::pow(10.0, SNR_db / 10.0);
  double noise_power = signal_power / SNR_linear;
  double noise_std_dev = std::sqrt(noise_power);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> awgn(0.0, noise_std_dev);

  const std::array<double, 9> b_saw = {
      5.6446553633438005e-03,  0.0000000000000000e+00, -2.2578621453375202e-02,
      0.0000000000000000e+00,  3.3867932180062801e-02, 0.0000000000000000e+00,
      -2.2578621453375202e-02, 0.0000000000000000e+00, 5.6446553633438005e-03};

  const std::array<double, 9> a_saw = {
      1.0000000000000000e+00,  -1.9984014443252818e-15, 2.2933238326639129e+00,
      -3.8302694349567901e-15, 2.1950664369912758e+00,  -2.7755575615628914e-15,
      9.8393582539528601e-01,  -8.6736173798840355e-16, 1.7250770688142397e-01};

  IIRFilter saw_filter(b_saw, a_saw);

  uint64_t total_samples = static_cast<uint64_t>(T_sim * fs);
  const double pi = 3.14159265358979323846;

  std::cout << "Генерация отсчетов (всего: " << total_samples << ")..."
            << std::endl;

  std::vector<uint8_t> signal_buffer;
  signal_buffer.reserve(total_samples);

  for (uint64_t n = 0; n < total_samples; ++n) {
    double t = (double)n / fs;

    double code_phase =
        (t * f_chip - code_shift_chips) + (f_chip / f_L1) * (f_doppler * t + 0.5 * f_dot * t * t);

    long long abs_chip = static_cast<long long>(std::floor(code_phase));
    int chip_index = abs_chip % 1023;
    if (chip_index < 0)
      chip_index += 1023;

    int nav_index = static_cast<int>(t / 0.020);
    if (nav_index >= nav_bpsk.size()) {
      nav_index = nav_bpsk.size() - 1;
    }

    double carrier_phase =
        2.0 * pi * (f_if * t + f_doppler * t + 0.5 * f_dot * t * t);
    double carrier = std::cos(carrier_phase);

    double clean_signal = prn_bpsk[chip_index] * nav_bpsk[nav_index] * carrier;
    double noisy_signal = clean_signal + awgn(gen);
    double filtered_signal = saw_filter.process(noisy_signal);

    uint8_t adc_1bit = (filtered_signal >= 0.0) ? 1 : 0;

    signal_buffer.push_back(adc_1bit);
  }

  std::cout << "Сохранение файла на диск..." << std::endl;
  std::ofstream out_file("gps_if_signal.bin", std::ios::binary);
  if (!out_file) {
    std::cerr << "Ошибка: не удалось создать файл!" << std::endl;
    return 1;
  }

  out_file.write(reinterpret_cast<const char *>(signal_buffer.data()),
                 signal_buffer.size());
  out_file.close();

  std::cout << "Готово! Сигнал успешно сохранен. Размер: "
            << signal_buffer.size() / (1024 * 1024) << " МБ." << std::endl;

  return 0;
}
