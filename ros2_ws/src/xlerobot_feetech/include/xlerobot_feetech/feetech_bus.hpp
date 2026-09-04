#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xlerobot_feetech
{

struct WheelVelocitySteps
{
  int16_t left = 0;
  int16_t right = 0;
};

struct PositionCommand
{
  uint8_t id = 0;
  int16_t position = 0;
  uint16_t speed = 0;
  uint8_t acceleration = 0;
};

class FeetechBus
{
public:
  virtual ~FeetechBus() = default;

  virtual bool connect(const std::string & port, int baudrate) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;
  virtual bool ping(uint8_t id) = 0;
  virtual bool initVelocityMotor(uint8_t id, bool enable_torque) = 0;
  virtual bool initPositionMotor(uint8_t id, bool enable_torque) = 0;
  virtual bool enableTorque(uint8_t id, bool enable) = 0;
  virtual bool syncWriteVelocity(
    uint8_t left_id,
    int16_t left_steps,
    uint8_t right_id,
    int16_t right_steps,
    uint8_t acceleration) = 0;
  virtual std::optional<WheelVelocitySteps> syncReadVelocity(
    uint8_t left_id, uint8_t right_id) = 0;
  virtual bool syncWritePosition(const std::vector<PositionCommand> & commands) = 0;
  virtual bool syncReadPositions(
    const std::vector<uint8_t> & ids, std::vector<int> & positions) = 0;
  virtual std::optional<int> readPosition(uint8_t id) = 0;
};

}  // namespace xlerobot_feetech
