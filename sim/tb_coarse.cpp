#include "Vtop_acqu.h"
#include "detector_cfar.hpp" // ПОДКЛЮЧАЕМ НАШ НОВЫЙ ДЕТЕКТОР
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <verilated.h>

inline int sign_ext_28(uint32_t v) {
  return (v & 0x08000000) ? (v | 0xF0000000) : v;
}
inline int sign_ext_18(uint32_t v) {
  return (v & 0x00020000) ? (v | 0xFFFC0000) : v;
}

int32_t calc_doppler_step(int hz) {
  return static_cast<int32_t>(hz * 4294967296.0 / 3819200.0);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Vtop_acqu *top = new Vtop_acqu;

  std::ifstream in_file("bin/gps_if_signal.bin",
                        std::ios::binary | std::ios::ate);
  if (!in_file.is_open()) {
    std::cerr << "Error: Cannot open gps_if_signal.bin." << std::endl;
    return 1;
  }

  std::streamsize file_size = in_file.tellg();

  in_file.seekg(0, std::ios::beg);

  std::vector<uint8_t> adc_data(file_size);
  std::cout << "Loading binary ADC data into memory..." << std::endl;

  if (in_file.read(reinterpret_cast<char *>(adc_data.data()), file_size)) {
    std::cout << "Loaded " << file_size << " samples successfully."
              << std::endl;
  } else {
    std::cerr << "Error: Failed to read the binary file." << std::endl;
    return 1;
  }
  in_file.close();

  std::vector<int32_t> fft_re, fft_im, mix_re, mix_im;
  std::vector<uint64_t> power_2d;

  fft_re.reserve(4096);
  fft_im.reserve(4096);
  mix_re.reserve(4096);
  mix_im.reserve(4096);
  power_2d.reserve(21 * 4096);

  std::vector<int> dopplers;
  for (int d = -5000; d <= 5000; d += 500) {
    dopplers.push_back(d);
  }

  std::cout << "Starting 2D Doppler Search: -5 kHz to +5 kHz..." << std::endl;

  bool global_signal_found = false;
  int best_doppler = 0;
  uint32_t best_shift = 0;
  uint64_t best_power = 0;
  double best_snr = 0.0;
  uint64_t best_threshold = 0;

  for (int doppler_hz : dopplers) {
    top->clk = 0;
    top->rst = 1;
    top->eval();
    for (int i = 0; i < 10; ++i) {
      top->clk = !top->clk;
      top->eval();
    }
    top->rst = 0;

    top->phase_step = 0x40000000;
    top->doppler_step = calc_doppler_step(doppler_hz);

    int adc_idx = 0;
    int points_collected = 0;
    bool first_cycle = true;

    top->start_capture = 1;

    while (!Verilated::gotFinish() && points_collected < 4096) {

      top->clk = 1;
      top->eval();

      if (doppler_hz == 0) {
        if (top->fft_ready && fft_re.size() < 4096) {
          fft_re.push_back(sign_ext_28(top->fft_out_re));
          fft_im.push_back(sign_ext_28(top->fft_out_im));
        }
        if (top->mix_out_valid && mix_re.size() < 4096) {
          mix_re.push_back(sign_ext_18(top->mix_out_re));
          mix_im.push_back(sign_ext_18(top->mix_out_im));
        }
      }

      if (top->peak_ready) {
        power_2d.push_back(top->power_out);
        points_collected++;
      }

      top->clk = 0;
      if (first_cycle) {
        top->start_capture = 0;
        first_cycle = false;
      }

      top->adc_input = (adc_idx < adc_data.size()) ? adc_data[adc_idx++] : 0;
      top->eval();
    }

    std::vector<uint64_t> current_frame(power_2d.end() - 4096, power_2d.end());
    DetectionResult det = apply_neyman_pearson(current_frame, 1e-5);

    std::cout << "  Doppler " << doppler_hz << " Hz -> Done." << std::endl;

    if (det.signal_found && det.max_power > best_power) {
      best_power = det.max_power;
      best_doppler = doppler_hz;
      best_shift = det.peak_index;
      best_snr = det.snr_db;
      best_threshold = det.threshold;
      global_signal_found = true;
    }
  }
  std::cout << "\n=======================================================\n";
  if (global_signal_found) {
    std::cout << ">>> СИГНАЛ ОБНАРУЖЕН (Критерий Неймана-Пирсона) <<<\n";
    std::cout << " Найденный Доплер : " << best_doppler << " Гц\n";
    std::cout << " Сдвиг кода       : " << best_shift << " отсчетов\n";
    std::cout << " Мощность пика    : " << best_power << "\n";
    std::cout << " Порог CFAR       : " << best_threshold << "\n";
    std::cout << " Оценка SNR       : " << best_snr << " dB\n";
  } else {
    std::cout << ">>> РЕЗУЛЬТАТЫ ПОИСКА <<<\n";
    std::cout << " Сигнал НЕ обнаружен в эфире (максимумы не пробили порог).\n";
  }
  std::cout << "=======================================================\n\n";

  std::cout << "Saving raw binary data..." << std::endl;

  std::ofstream p_file("bin/power_2d.bin", std::ios::binary);
  p_file.write(reinterpret_cast<const char *>(power_2d.data()),
               power_2d.size() * sizeof(uint64_t));
  p_file.close();

  std::ofstream s_file("bin/spectra.bin", std::ios::binary);
  s_file.write(reinterpret_cast<const char *>(fft_re.data()),
               fft_re.size() * sizeof(int32_t));
  s_file.write(reinterpret_cast<const char *>(fft_im.data()),
               fft_im.size() * sizeof(int32_t));
  s_file.write(reinterpret_cast<const char *>(mix_re.data()),
               mix_re.size() * sizeof(int32_t));
  s_file.write(reinterpret_cast<const char *>(mix_im.data()),
               mix_im.size() * sizeof(int32_t));
  s_file.close();

  std::cout << "Simulation finished successfully!" << std::endl;
  delete top;
  return 0;
}
