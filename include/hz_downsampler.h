/*
 * @Author: juling julinger.qq.com
 * @Date: 2024-09-13 10:27:51
 * @LastEditors: juling julinger.qq.com
 * @LastEditTime: 2024-12-06 16:03:31
 * @FilePath: /perception_ws/src/perception_test/include/hz_downsampler.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
  bool Pulse();

 private:
  // The cycle time.
  std::chrono::nanoseconds tc_;
  std::chrono::steady_clock::time_point last_time_;
};

#endif  // AUTO_DOWNSAMPLER_H