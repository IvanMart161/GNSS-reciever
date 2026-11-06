#include "Vaxi4_lite_acqu.h"
#include "detector_cfar.hpp" 
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <verilated.h>

namespace {
constexpr uint32_t REG_CTRL = 0x00;
constexpr uint32_t REG_PHASE_STEP = 0x08;
constexpr uint32_t REG_DOPPLER_STEP = 0x0C;
constexpr int AXI_TIMEOUT_CYCLES = 100;

void axi_write(Vaxi4_lite_acqu *top, uint32_t address, uint32_t data) {
  bool address_sent = false;
  bool data_sent = false;

  top->s_axi_awaddr = address;
  top->s_axi_wdata = data;
  top->s_axi_wstrb = 0xF;
  top->s_axi_bready = 0;

  for (int cycle = 0;
       cycle < AXI_TIMEOUT_CYCLES && (!address_sent || !data_sent); ++cycle) {
    top->s_axi_awvalid = !address_sent;
    top->s_axi_wvalid = !data_sent;
    top->aclk = 0;
    top->eval();
    bool address_handshake = !address_sent && top->s_axi_awready;
    bool data_handshake = !data_sent && top->s_axi_wready;
    top->aclk = 1;
    top->eval();
    address_sent = address_sent || address_handshake;
    data_sent = data_sent || data_handshake;
  }

  top->s_axi_awvalid = 0;
  top->s_axi_wvalid = 0;
  if (!address_sent || !data_sent) {
    throw std::runtime_error("AXI write request timed out waiting for AW/W");
  }

  top->s_axi_bready = 1;
  for (int cycle = 0; cycle < AXI_TIMEOUT_CYCLES; ++cycle) {
    top->aclk = 0;
    top->eval();
    bool response_valid = top->s_axi_bvalid;
    uint8_t response = top->s_axi_bresp;
    top->aclk = 1;
    top->eval();
    if (response_valid) {
      top->s_axi_bready = 0;
      if (response != 0) {
        throw std::runtime_error("AXI write returned an error response");
      }
      return;
    }
  }
  top->s_axi_bready = 0;
  throw std::runtime_error("AXI write timed out waiting for B");
}

uint32_t axi_read(Vaxi4_lite_acqu *top, uint32_t address) {
  top->s_axi_araddr = address;
  top->s_axi_arvalid = 1;
  top->s_axi_rready = 0;
  bool address_sent = false;

  for (int cycle = 0; cycle < AXI_TIMEOUT_CYCLES && !address_sent; ++cycle) {
    top->aclk = 0;
    top->eval();
    bool address_handshake = top->s_axi_arready;
    top->aclk = 1;
    top->eval();
    address_sent = address_handshake;
  }
  top->s_axi_arvalid = 0;
  if (!address_sent) {
    throw std::runtime_error("AXI read timed out waiting for AR");
  }

  top->s_axi_rready = 1;
  for (int cycle = 0; cycle < AXI_TIMEOUT_CYCLES; ++cycle) {
    top->aclk = 0;
    top->eval();
    bool response_valid = top->s_axi_rvalid;
    uint32_t data = top->s_axi_rdata;
    uint8_t response = top->s_axi_rresp;
    top->aclk = 1;
    top->eval();
    if (response_valid) {
      top->s_axi_rready = 0;
      if (response != 0) {
        throw std::runtime_error("AXI read returned an error response");
      }
      return data;
    }
  }
  top->s_axi_rready = 0;
  throw std::runtime_error("AXI read timed out waiting for R");
}

void reset_dut(Vaxi4_lite_acqu *top) {
  top->aclk = 0;
  top->aresetn = 1;
  top->eval();
  top->aresetn = 0;
  top->eval();
  for (int cycle = 0; cycle < 10; ++cycle) {
    top->aclk = 0;
    top->eval();
    top->aclk = 1;
    top->eval();
  }
  top->aclk = 0;
  top->aresetn = 1;
  top->eval();
}
} // namespace

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
  Vaxi4_lite_acqu *top = new Vaxi4_lite_acqu;

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
  power_2d.reserve(201 * 4096);

  std::vector<int> dopplers;
  for (int d = -50000; d <= 50000; d += 500) {
    dopplers.push_back(d);
  }

  std::cout << "Starting 2D Doppler Search: -50 kHz to +50 kHz..." << std::endl;

  bool global_signal_found = false;
  int best_doppler = 0;
  uint32_t best_shift = 0;
  uint64_t best_power = 0;
  double best_snr = 0.0;
  uint64_t best_threshold = 0;

  for (int doppler_hz : dopplers) {
    reset_dut(top);
    axi_write(top, REG_PHASE_STEP, 0x40000000);
    if (axi_read(top, REG_PHASE_STEP) != 0x40000000) {
      throw std::runtime_error("AXI phase-step register readback mismatch");
    }
    axi_write(top, REG_DOPPLER_STEP,
              static_cast<uint32_t>(calc_doppler_step(doppler_hz)));

    int adc_idx = 0;
    int points_collected = 0;
    axi_write(top, REG_CTRL, 1);

    while (!Verilated::gotFinish() && points_collected < 4096) {
      top->aclk = 0;
      top->adc_input = (adc_idx < adc_data.size()) ? adc_data[adc_idx++] : 0;
      top->eval();
      top->aclk = 1;
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
