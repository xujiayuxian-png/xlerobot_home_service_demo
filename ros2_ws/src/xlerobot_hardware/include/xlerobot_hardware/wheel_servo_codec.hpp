#pragma once

#include <cstdint>

namespace xlerobot_hardware
{

struct EncodedWheelPair
{
  int16_t left_steps{0};
  int16_t right_steps{0};
  bool scaled{false};
};

class WheelServoCodec
{
public:
  WheelServoCodec(
    bool invert_left,
    bool invert_right,
    bool feedback_invert_left,
    bool feedback_invert_right,
    int max_raw_velocity);

  EncodedWheelPair encode(double left_radps, double right_radps) const;
  double decode_left(int16_t steps) const;
  double decode_right(int16_t steps) const;

private:
  double decode(int16_t steps, bool invert) const;

  bool invert_left_;
  bool invert_right_;
  bool feedback_invert_left_;
  bool feedback_invert_right_;
  int max_raw_velocity_;
};

}  // namespace xlerobot_hardware
