#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Vtracker_channel.h"
#include "Vtracker_channel___024root.h"
#include "verilated.h"

#include "gnss_demodulator.hpp"
#include "gnss_parser.hpp"
#include "gnss_tracking.hpp"

#pragma pack(push, 1)
struct TrackingRecord {
  uint32_t time_ms;
  int32_t i_p;
  int32_t q_p;
  int32_t state;
  float fine_tune_hz;
  int32_t nco_adjust_reg;
  float fine_tune_code_hz;
  int32_t code_adjust_reg;
};

struct TactRecord {
  uint32_t tact;
  int32_t i_sum;
  int32_t q_sum;
};
#pragma pack(pop)

int32_t sign_ext24(uint32_t val) {
  if (val & 0x800000)
    return static_cast<int32_t>(val | 0xFF000000);
  return static_cast<int32_t>(val);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Vtracker_channel *dut = new Vtracker_channel;

  float coarse_doppler_hz = 0.0f;
  int code_shift_adc = 0;

  if (argc > 1) {
    coarse_doppler_hz = std::stof(argv[1]);
  }
  if (argc > 2) {
    // int shift = std::stoi(argv[2]) * 10 - 18;
    uint32_t coarse_shift_bins = std::stoi(argv[2]);
    
    // Компенсируем конвейерную задержку модуля БПФ в Vtop_acqu (~100 тактов)
    // Эта задержка добавляется аппаратно к индексу пика
    if (coarse_shift_bins >= 100) {
        coarse_shift_bins -= 100;
    } else {
        coarse_shift_bins = 0; // Защита от underflow
    }

    // Переводим бины из сетки 4096 (Acquisition) в отсчеты АЦП 38192 (Tracking)
    code_shift_adc = static_cast<int>(std::round(coarse_shift_bins * (38192.0 / 4096.0)));
    
    std::cout << "--- Initialization ---" << std::endl;
    std::cout << "Compensated FFT bins: " << coarse_shift_bins << std::endl;
    std::cout << "ADC Start Offset    : " << code_shift_adc << " samples" << std::endl;
  }

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

  std::ofstream bin_file("bin/tracking_results.bin", std::ios::binary);
  std::ofstream tact_bin("bin/correlation_tact.bin", std::ios::binary);
  if (!bin_file.is_open() || !tact_bin.is_open()) {
    std::cerr << "Error: Cannot create output binary files" << std::endl;
    return 1;
  }

  const float F_SYS = 38192000.0f;
  const float F_IF = 9548000.0f;

  dut->rst = 1;
  dut->clk = 0;
  dut->eval();
  dut->clk = 1;
  dut->eval();
  dut->rst = 0;

  double hz_to_nco = 4294967296.0 / F_SYS;
  dut->carrier_phase_step =
      static_cast<uint32_t>(std::round((F_IF + coarse_doppler_hz) * hz_to_nco));
  dut->carrier_phase_adjust = 0;

  dut->code_phase_step = 115043767;
  dut->code_phase_adjust = 0;

  dut->s1 = 2;
  dut->s2 = 6;

  TrackingChannelController controller(F_SYS, 15.0f, 20.0f, 0.001f);
  NavDataDemodulator demodulator;
  NavDataParser parser;

  uint32_t ms_counter = 0;
  uint32_t tact = 0;

  std::cout << "=== Starting CLOSED-LOOP Simulation ===" << std::endl;

  for (size_t adc_idx = code_shift_adc; adc_idx < adc_data.size(); ++adc_idx) {

    dut->adc_data_in = adc_data[adc_idx];

    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();

    int current_i =
        sign_ext24(dut->rootp->tracker_channel__DOT__u_accum__DOT__sum_i_p);
    int current_q =
        sign_ext24(dut->rootp->tracker_channel__DOT__u_accum__DOT__sum_q_p);

    TactRecord trec;
    trec.tact = tact;
    trec.i_sum = current_i;
    trec.q_sum = current_q;
    tact_bin.write(reinterpret_cast<const char *>(&trec), sizeof(TactRecord));

    tact++;

    if (dut->dump_valid) {
      ms_counter++;

      int32_t i_e = sign_ext24(dut->acc_i_e);
      int32_t q_e = sign_ext24(dut->acc_q_e);
      int32_t i_p = sign_ext24(dut->acc_i_p);
      int32_t q_p = sign_ext24(dut->acc_q_p);
      int32_t i_l = sign_ext24(dut->acc_i_l);
      int32_t q_l = sign_ext24(dut->acc_q_l);

      TrackingResponse response =
          controller.process_sample(i_e, q_e, i_p, q_p, i_l, q_l); // [NEW]

      bool is_locked = (controller.get_state() == STATE_EKF_TRACKING);
      if (demodulator.process_ms(i_p, is_locked)) {
        uint32_t new_word = demodulator.get_latest_word();
        parser.process_word(new_word);
      }

      dut->carrier_phase_adjust = response.carrier_nco_adjust;
      dut->code_phase_adjust = response.code_nco_adjust;

      TrackingRecord rec;
      rec.time_ms = ms_counter;
      rec.i_p = i_p;
      rec.q_p = q_p;
      rec.state = static_cast<int32_t>(controller.get_state());
      rec.fine_tune_hz = controller.get_fine_tune_hz();
      rec.nco_adjust_reg = response.carrier_nco_adjust;
      rec.fine_tune_code_hz = response.fine_tune_code_hz;
      rec.code_adjust_reg = response.code_nco_adjust;

      bin_file.write(reinterpret_cast<const char *>(&rec),
                     sizeof(TrackingRecord));

      if (ms_counter % 100 == 0) {
        std::cout << "ms: " << ms_counter << " | State: " << rec.state
                  << " | I_P: " << i_p << "\t Q_P: " << q_p
                  << " | Tune: " << rec.fine_tune_hz << " Hz\n";
      }
    }
  }

  tact_bin.close();
  bin_file.close();
  dut->final();
  delete dut;

  demodulator.print_words();

  std::cout << "Simulation finished! Check tracking_results.bin" << std::endl;
  return 0;
}
