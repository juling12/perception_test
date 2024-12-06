#ifndef HZ_DOWNSAMPLER_H
#define HZ_DOWNSAMPLER_H
#include <chrono>
#include <iostream>
class HzDownSampler {
 public:
  HzDownSampler(int expected_hz) {
    tc_ = std::chrono::nanoseconds(static_cast<long>(1.0 / expected_hz * 1e9));
    last_time_ = std::chrono::steady_clock::now();
  }

  HzDownSampler(const HzDownSampler&) = delete;
  HzDownSampler& operator=(const HzDownSampler&) = delete;

  ~HzDownSampler(){};
  // Returns true if this pulse should result in an sample.
  bool Pluse();

 private:
  // The cycle time.
  std::chrono::nanoseconds tc_;
  std::chrono::steady_clock::time_point last_time_;
};

#endif  // AUTO_DOWNSAMPLER_H