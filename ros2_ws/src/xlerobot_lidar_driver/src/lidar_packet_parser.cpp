#include "xlerobot_lidar_driver/lidar_packet_parser.hpp"

#include "xlerobot_lidar_driver/lidar_protocol.hpp"

namespace xlerobot_lidar_driver
{

LidarPacketParser::LidarPacketParser(bool checksum_enabled)
: checksum_enabled_(checksum_enabled)
{
}

void LidarPacketParser::setChecksumEnabled(bool enabled)
{
  checksum_enabled_ = enabled;
}

std::optional<LidarPacket> LidarPacketParser::processByte(uint8_t byte)
{
  switch (state_) {
    case State::kWaitHeader1:
      if (byte == 0xAA) {
        state_ = State::kWaitHeader2;
        packet_buffer_.clear();
        packet_buffer_.push_back(byte);
      }
      break;

    case State::kWaitHeader2:
      if (byte == 0x55) {
        state_ = State::kReadMeta;
        packet_buffer_.push_back(byte);
      } else if (byte == 0xAA) {
        packet_buffer_.clear();
        packet_buffer_.push_back(byte);
      } else {
        reset();
      }
      break;

    case State::kReadMeta:
      packet_buffer_.push_back(byte);
      if (packet_buffer_.size() == 4) {
        target_packet_size_ = LidarProtocol::expectedPacketSize(packet_buffer_[3]);
        state_ = State::kReadPayload;
      }
      break;

    case State::kReadPayload:
      packet_buffer_.push_back(byte);
      if (packet_buffer_.size() == target_packet_size_) {
        std::optional<LidarPacket> packet;
        if (!checksum_enabled_ || LidarProtocol::isChecksumValid(packet_buffer_)) {
          packet = LidarProtocol::parsePacket(packet_buffer_);
        } else {
          invalid_checksum_count_++;
          last_expected_checksum_ = LidarProtocol::bytesToUint16(packet_buffer_, 8);
          last_actual_checksum_ = LidarProtocol::calculateChecksum(packet_buffer_);
        }
        reset();
        return packet;
      }
      break;
  }

  return std::nullopt;
}

uint64_t LidarPacketParser::invalidChecksumCount() const
{
  return invalid_checksum_count_;
}

uint16_t LidarPacketParser::lastExpectedChecksum() const
{
  return last_expected_checksum_;
}

uint16_t LidarPacketParser::lastActualChecksum() const
{
  return last_actual_checksum_;
}

void LidarPacketParser::reset()
{
  state_ = State::kWaitHeader1;
  packet_buffer_.clear();
  target_packet_size_ = 0;
}

}  // namespace xlerobot_lidar_driver
