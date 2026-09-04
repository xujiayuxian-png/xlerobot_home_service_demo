#include "xlerobot_hardware/calibrated_servo_codec.hpp"

#include <algorithm>
#include <cmath>

namespace xlerobot_hardware
{

namespace
{
constexpr double kStepsPerRadian = 4096.0 / (2.0 * M_PI);
constexpr double kTolerance = 1.0e-9;
}  // namespace

CalibratedServoCodec::CalibratedServoCodec(PositionServoCalibration calibration)
: calibration_(calibration)
{
}

bool CalibratedServoCodec::valid() const
{
  return (calibration_.direction == 1 || calibration_.direction == -1) &&
         calibration_.raw_min >= 0 && calibration_.raw_min <= calibration_.raw_max &&
         calibration_.raw_max <= 4095 &&
         std::isfinite(calibration_.position_min) &&
         std::isfinite(calibration_.position_max) &&
         calibration_.position_min <= calibration_.position_max;
}

std::optional<int> CalibratedServoCodec::encode(double position) const
{
  if (!valid() || !std::isfinite(position) ||
    position < calibration_.position_min - kTolerance ||
    position > calibration_.position_max + kTolerance)
  {
    return std::nullopt;
  }
  const int raw = static_cast<int>(std::lround(
      calibration_.offset + calibration_.direction * position * kStepsPerRadian));
  // The calibrated raw range is the final motor boundary.  A valid ROS joint
  // command can map a few steps beyond that range because the URDF limits and
  // the measured servo range are intentionally separate artifacts.  Preserve
  // the established XLeRobot hardware behavior by saturating at the measured
  // raw boundary instead of faulting the entire shared bus.
  return std::clamp(raw, calibration_.raw_min, calibration_.raw_max);
}

std::optional<double> CalibratedServoCodec::decode(int raw_position) const
{
  if (!valid() || raw_position < calibration_.raw_min || raw_position > calibration_.raw_max) {
    return std::nullopt;
  }
  const double position = calibration_.direction *
    (raw_position - calibration_.offset) / kStepsPerRadian;
  if (position < calibration_.position_min - kTolerance ||
    position > calibration_.position_max + kTolerance)
  {
    return std::nullopt;
  }
  return position;
}

std::optional<double> CalibratedServoCodec::decode_observation(int raw_position) const
{
  if (!valid() || raw_position < 0 || raw_position > 4095) {
    return std::nullopt;
  }
  return calibration_.direction *
         (raw_position - calibration_.offset) / kStepsPerRadian;
}

}  // namespace xlerobot_hardware
