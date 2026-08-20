#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

struct DetectionResult {
  bool signal_found;
  uint32_t peak_index;
  uint64_t max_power;
  uint64_t threshold;
  double snr_db;
};

inline DetectionResult
apply_neyman_pearson(const std::vector<uint64_t> &power_frame,
                     double p_fa = 1e-5) {
  DetectionResult result;
  result.signal_found = false;

  if (power_frame.empty())
    return result;

  int x;

  uint64_t max_power = 0;
  uint64_t sum_power = 0;
  uint32_t max_index = 0;

  for (size_t i = 0; i < power_frame.size(); ++i) {
    if (power_frame[i] > max_power) {
      max_power = power_frame[i];
      max_index = static_cast<uint32_t>(i);
    }
    sum_power += power_frame[i];
  }

  double mean_noise_power =
      static_cast<double>(sum_power - max_power) / (power_frame.size() - 1);

  if (mean_noise_power <= 0.0)
    mean_noise_power = 1.0;

  double alpha = -std::log(p_fa);
  double threshold_calc = mean_noise_power * alpha;

  result.threshold = static_cast<uint64_t>(threshold_calc);
  result.max_power = max_power;
  result.peak_index = max_index;

  result.snr_db =
      10.0 * std::log10(static_cast<double>(max_power) / mean_noise_power);

  if (max_power >= result.threshold) {
    result.signal_found = true;
  }

  return result;
}
