#include "xlerobot_feetech/scservo_feetech_bus.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <scservo/SCServo.h>

namespace xlerobot_feetech
{

namespace
{
constexpr uint8_t kSpeedDataLen = 2;
constexpr uint8_t kPositionDataLen = 2;
}  // namespace

ScServoFeetechBus::ScServoFeetechBus()
: servo_(std::make_unique<SMS_STS>())
{
  // SCServo_Linux leaves its sync-read bookkeeping uninitialized.  Its first
  // syncReadBegin() conditionally deletes syncReadRxBuff, so an indeterminate
  // pointer can crash before any packet is sent.  Own that initialization at
  // the wrapper boundary instead of carrying a fork of the upstream submodule.
  servo_->syncReadRxPacketIndex = 0;
  servo_->syncReadRxPacketLen = 0;
  servo_->syncReadRxPacket = nullptr;
  servo_->syncReadRxBuff = nullptr;
  servo_->syncReadRxBuffLen = 0;
  servo_->syncReadRxBuffMax = 0;
}

ScServoFeetechBus::~ScServoFeetechBus()
{
  disconnect();
}

bool ScServoFeetechBus::connect(const std::string & port, int baudrate)
{
  disconnect();
  connected_ = servo_->begin(baudrate, port.c_str());
  return connected_;
}

void ScServoFeetechBus::disconnect()
{
  if (connected_) {
    servo_->end();
    connected_ = false;
  }
}

bool ScServoFeetechBus::isConnected() const
{
  return connected_;
}

bool ScServoFeetechBus::ping(uint8_t id)
{
  return connected_ && servo_->Ping(id) >= 0;
}

bool ScServoFeetechBus::initVelocityMotor(uint8_t id, bool enable_torque)
{
  return connected_ &&
         servo_->InitMotor(id, SMS_STS_MODE_WHEEL_CLOSED, enable_torque ? 1 : 0) == 1;
}

bool ScServoFeetechBus::initPositionMotor(uint8_t id, bool enable_torque)
{
  return connected_ && servo_->InitMotor(id, SMS_STS_MODE_SERVO, enable_torque ? 1 : 0) == 1;
}

bool ScServoFeetechBus::enableTorque(uint8_t id, bool enable)
{
  return connected_ && servo_->EnableTorque(id, enable ? 1 : 0) == 1;
}

bool ScServoFeetechBus::syncWriteVelocity(
  uint8_t left_id,
  int16_t left_steps,
  uint8_t right_id,
  int16_t right_steps,
  uint8_t acceleration)
{
  if (!connected_) {
    return false;
  }
  std::array<uint8_t, 2> ids{left_id, right_id};
  std::array<int16_t, 2> speeds{left_steps, right_steps};
  std::array<uint8_t, 2> accelerations{acceleration, acceleration};
  servo_->SyncWriteSpe(ids.data(), ids.size(), speeds.data(), accelerations.data());
  return true;
}

std::optional<WheelVelocitySteps> ScServoFeetechBus::syncReadVelocity(
  uint8_t left_id, uint8_t right_id)
{
  if (!connected_) {
    return std::nullopt;
  }

  std::array<uint8_t, 2> ids{left_id, right_id};
  std::array<uint8_t, kSpeedDataLen> data{};
  servo_->syncReadBegin(ids.size(), kSpeedDataLen);
  const int received = servo_->syncReadPacketTx(
    ids.data(), ids.size(), SMS_STS_PRESENT_SPEED_L, kSpeedDataLen);
  if (received <= 0) {
    servo_->syncReadEnd();
    return std::nullopt;
  }
  if (servo_->syncReadPacketRx(left_id, data.data()) != kSpeedDataLen) {
    servo_->syncReadEnd();
    return std::nullopt;
  }
  const int left = servo_->syncReadRxPacketToWrod(SMS_STS_DIRECTION_BIT_POS);
  if (servo_->syncReadPacketRx(right_id, data.data()) != kSpeedDataLen) {
    servo_->syncReadEnd();
    return std::nullopt;
  }
  const int right = servo_->syncReadRxPacketToWrod(SMS_STS_DIRECTION_BIT_POS);
  servo_->syncReadEnd();

  if (left < std::numeric_limits<int16_t>::min() ||
    left > std::numeric_limits<int16_t>::max() ||
    right < std::numeric_limits<int16_t>::min() ||
    right > std::numeric_limits<int16_t>::max())
  {
    return std::nullopt;
  }
  return WheelVelocitySteps{static_cast<int16_t>(left), static_cast<int16_t>(right)};
}

bool ScServoFeetechBus::syncWritePosition(const std::vector<PositionCommand> & commands)
{
  if (!connected_ || commands.empty()) {
    return false;
  }
  std::vector<uint8_t> ids;
  std::vector<int16_t> positions;
  std::vector<uint16_t> speeds;
  std::vector<uint8_t> accelerations;
  ids.reserve(commands.size());
  positions.reserve(commands.size());
  speeds.reserve(commands.size());
  accelerations.reserve(commands.size());
  for (const auto & command : commands) {
    ids.push_back(command.id);
    positions.push_back(command.position);
    speeds.push_back(command.speed);
    accelerations.push_back(command.acceleration);
  }
  servo_->SyncWritePosEx(
    ids.data(), ids.size(), positions.data(), speeds.data(), accelerations.data());
  return true;
}

bool ScServoFeetechBus::syncReadPositions(
  const std::vector<uint8_t> & ids, std::vector<int> & positions)
{
  if (!connected_ || ids.empty()) {
    return false;
  }
  positions.assign(ids.size(), -1);
  std::vector<uint8_t> id_list = ids;
  servo_->syncReadBegin(id_list.size(), kPositionDataLen);
  const int received = servo_->syncReadPacketTx(
    id_list.data(), id_list.size(), SMS_STS_PRESENT_POSITION_L, kPositionDataLen);
  if (received <= 0) {
    servo_->syncReadEnd();
    return false;
  }
  bool ok = true;
  std::array<uint8_t, kPositionDataLen> data{};
  for (size_t index = 0; index < id_list.size(); ++index) {
    if (servo_->syncReadPacketRx(id_list[index], data.data()) != kPositionDataLen) {
      ok = false;
      continue;
    }
    positions[index] = servo_->syncReadRxPacketToWrod(SMS_STS_DIRECTION_BIT_POS);
  }
  servo_->syncReadEnd();
  return ok;
}

std::optional<int> ScServoFeetechBus::readPosition(uint8_t id)
{
  if (!connected_) {
    return std::nullopt;
  }
  const int position = servo_->ReadPos(id);
  return position < 0 ? std::nullopt : std::optional<int>(position);
}

}  // namespace xlerobot_feetech
