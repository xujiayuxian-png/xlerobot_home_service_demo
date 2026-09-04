#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "xlerobot_feetech/feetech_bus.hpp"

class SMS_STS;

namespace xlerobot_feetech
{

class ScServoFeetechBus : public FeetechBus
{
public:
  ScServoFeetechBus();
  ~ScServoFeetechBus() override;

  ScServoFeetechBus(const ScServoFeetechBus &) = delete;
  ScServoFeetechBus & operator=(const ScServoFeetechBus &) = delete;

  bool connect(const std::string & port, int baudrate) override;
  void disconnect() override;
  bool isConnected() const override;
  bool ping(uint8_t id) override;
  bool initVelocityMotor(uint8_t id, bool enable_torque) override;
  bool initPositionMotor(uint8_t id, bool enable_torque) override;
  bool enableTorque(uint8_t id, bool enable) override;
  bool syncWriteVelocity(
    uint8_t left_id,
    int16_t left_steps,
    uint8_t right_id,
    int16_t right_steps,
    uint8_t acceleration) override;
  std::optional<WheelVelocitySteps> syncReadVelocity(
    uint8_t left_id, uint8_t right_id) override;
  bool syncWritePosition(const std::vector<PositionCommand> & commands) override;
  bool syncReadPositions(
    const std::vector<uint8_t> & ids, std::vector<int> & positions) override;
  std::optional<int> readPosition(uint8_t id) override;

private:
  std::unique_ptr<SMS_STS> servo_;
  bool connected_ = false;
};

}  // namespace xlerobot_feetech
