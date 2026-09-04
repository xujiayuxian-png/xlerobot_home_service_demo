#pragma once

#include <optional>

namespace xlerobot_hardware
{

struct PositionServoCalibration
{
  int offset = 2048;
  int direction = 1;
  int raw_min = 0;
  int raw_max = 4095;
  double position_min = 0.0;
  double position_max = 0.0;
};

class CalibratedServoCodec
{
public:
  explicit CalibratedServoCodec(PositionServoCalibration calibration);

  bool valid() const;
  std::optional<int> encode(double position) const;
  std::optional<double> decode(int raw_position) const;
  std::optional<double> decode_observation(int raw_position) const;

private:
  PositionServoCalibration calibration_;
};

}  // namespace xlerobot_hardware
