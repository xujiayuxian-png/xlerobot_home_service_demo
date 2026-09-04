#include "xlerobot_hardware/wheel_servo_codec.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xlerobot_hardware
{

namespace
{
constexpr double kStepsPerRad = 4096.0 / (2.0 * M_PI);
}

WheelServoCodec::WheelServoCodec(
  bool invert_left,
  bool invert_right,
  bool feedback_invert_left,
  bool feedback_invert_right,
  int max_raw_velocity)
: invert_left_(invert_left),
  invert_right_(invert_right),
  feedback_invert_left_(feedback_invert_left),
  feedback_invert_right_(feedback_invert_right),
  max_raw_velocity_(max_raw_velocity)
{
  if (max_raw_velocity_ <= 0 || max_raw_velocity_ > 0x7FFF) {
    throw std::invalid_argument("max_raw_velocity must be in [1, 32767]");
  }
}

EncodedWheelPair WheelServoCodec::encode(double left_radps, double right_radps) const
{
  double left = (invert_left_ ? -left_radps : left_radps) * kStepsPerRad;
  double right = (invert_right_ ? -right_radps : right_radps) * kStepsPerRad;
  const double largest = std::max(std::abs(left), std::abs(right));
  bool scaled = false;
  if (largest > static_cast<double>(max_raw_velocity_)) {
    const double scale = static_cast<double>(max_raw_velocity_) / largest;
    left *= scale;
    right *= scale;
    scaled = true;
  }
  return {
    static_cast<int16_t>(std::lround(left)),
    static_cast<int16_t>(std::lround(right)),
    scaled,
  };
}

double WheelServoCodec::decode_left(int16_t steps) const
{
  return decode(steps, feedback_invert_left_);
}

double WheelServoCodec::decode_right(int16_t steps) const
{
  return decode(steps, feedback_invert_right_);
}

double WheelServoCodec::decode(int16_t steps, bool invert) const
{
  const double radps = static_cast<double>(steps) / kStepsPerRad;
  return invert ? -radps : radps;
}

}  // namespace xlerobot_hardware
