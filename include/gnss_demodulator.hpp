#pragma once

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

enum DemodulatorState {
  STATE_SEARCHING_SYNC = 0,
  STATE_WAITING_FRAME = 1,
  STATE_DEMODULATING = 2
};

class NavDataDemodulator {
private:
  DemodulatorState state;

  uint32_t edge_histogram[20];
  int32_t prev_i_p;
  uint32_t sync_ms_counter;

  static constexpr uint32_t SYNC_DURATION_MS = 300;

  int32_t current_bit_sum;
  uint32_t ms_in_bit;

  uint8_t shift_reg;
  bool phase_inverted;

  uint32_t current_word;
  uint32_t bit_count;

  std::vector<uint32_t> decoded_words;

public:
  NavDataDemodulator()
      : state(STATE_SEARCHING_SYNC), prev_i_p(0), sync_ms_counter(0),
        current_bit_sum(0), ms_in_bit(0), shift_reg(0), phase_inverted(false),
        current_word(0), bit_count(0) {
    std::fill(std::begin(edge_histogram), std::end(edge_histogram), 0);
  }

  bool process_ms(int32_t i_p, bool is_locked) {
    if (!is_locked) {
      state = STATE_SEARCHING_SYNC;
      sync_ms_counter = 0;
      std::fill(std::begin(edge_histogram), std::end(edge_histogram), 0);
      current_bit_sum = 0;
      ms_in_bit = 0;
      prev_i_p = i_p;
      return false;
    }

    if (state == STATE_SEARCHING_SYNC) {
      bool is_sign_flipped =
          ((i_p > 0 && prev_i_p < 0) || (i_p < 0 && prev_i_p > 0));

      if (is_sign_flipped && sync_ms_counter > 0) {
        uint32_t phase_mod_20 = sync_ms_counter % 20;
        edge_histogram[phase_mod_20]++;
      }

      prev_i_p = i_p;
      sync_ms_counter++;

      if (sync_ms_counter >= SYNC_DURATION_MS) {
        uint32_t max_edges = 0;
        uint32_t bit_boundary_offset = 0;

        for (uint32_t i = 0; i < 20; ++i) {
          if (edge_histogram[i] > max_edges) {
            max_edges = edge_histogram[i];
            bit_boundary_offset = i;
          }
        }

        std::cout << "\n[DEMODULATOR] Битовая синхронизация найдена! Граница: "
                  << bit_boundary_offset << " мс\n";

        state = STATE_WAITING_FRAME;
        current_bit_sum = 0;
        ms_in_bit = 0;
        shift_reg = 0;
      }
      return false;
    }

    current_bit_sum += i_p;
    ms_in_bit++;

    if (ms_in_bit == 20) {
      uint8_t raw_bit = (current_bit_sum > 0) ? 0 : 1;
      current_bit_sum = 0;
      ms_in_bit = 0;

      if (state == STATE_WAITING_FRAME) {
        shift_reg = (shift_reg << 1) | raw_bit;

        uint8_t target_preamble = 0x8B;
        uint8_t target_preamble_inv = ~target_preamble;

        if (shift_reg == target_preamble || shift_reg == target_preamble_inv) {
          phase_inverted = (shift_reg == target_preamble_inv);
          state = STATE_DEMODULATING;

          current_word = 0x8B;
          bit_count = 8;

          std::cout << "[DEMODULATOR] Кадровая синхронизация (TLM Preamble "
                       "0x8B) найдена!\n";
          std::cout << "[DEMODULATOR] Фаза несущей: "
                    << (phase_inverted ? "180" : "0") << " градусов.\n";
        }
      } else if (state == STATE_DEMODULATING) {
        uint8_t actual_bit = phase_inverted ? (raw_bit ^ 1) : raw_bit;

        current_word = (current_word << 1) | actual_bit;
        bit_count++;

        if (bit_count == 30) {
          decoded_words.push_back(current_word);
          bit_count = 0;
          current_word = 0;
          return true;
        }
      }
    }
    return false;
  }

  uint32_t get_latest_word() const { return decoded_words.back(); }
  void print_words() const {
    std::cout << "\n======================================================\n";
    std::cout << "[DEMODULATOR] Принятые навигационные слова (HEX):\n";
    for (size_t i = 0; i < decoded_words.size(); ++i) {
      std::cout << "Слово " << i + 1 << ": 0x" << std::hex << std::uppercase
                << std::setfill('0') << std::setw(8) << decoded_words[i]
                << std::dec << "\n";
    }
    std::cout << "======================================================\n";
  }
};
