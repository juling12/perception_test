#include "hz_downsampler.h"
bool HzDownSampler::Pulse() {
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  auto time_min = last_time_ - tc_;
  auto time_max = last_time_ + tc_;
  if (time_max < now) {
    last_time_ = now + tc_;
    return true;
  }

  if (time_min <= now) {
    last_time_ += tc_;
    return true;
  }
  return false;
}