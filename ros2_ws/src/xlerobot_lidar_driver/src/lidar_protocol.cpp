#include "xlerobot_lidar_driver/lidar_protocol.hpp"

#include <cmath>
#include <stdexcept>

namespace xlerobot_lidar_driver
{

uint16_t LidarProtocol::bytesToUint16(const std::vector<uint8_t> & data, size_t index)
{
  if (index + 1 >= data.size()) {
    throw std::out_of_range("LidarProtocol::bytesToUint16 requires two bytes");
  }
  return (static_cast<uint16_t>(data[index + 1]) << 8) | data[index];
}

size_t LidarProtocol::expectedPacketSize(uint8_t lsn)
{
  return kPacketHeaderSize + static_cast<size_t>(lsn) * kSampleSize;
}

uint16_t LidarProtocol::calculateChecksum(const std::vector<uint8_t> & packet_data)
{
  if (packet_data.size() < kPacketHeaderSize) {
    return 0;
  }

  const uint8_t lsn = packet_data[3];
  if (packet_data.size() < expectedPacketSize(lsn)) {
    return 0;
  }

  uint16_t checksum = 0x55AA;
  checksum ^= bytesToUint16(packet_data, 2);
  checksum ^= bytesToUint16(packet_data, 4);
  checksum ^= bytesToUint16(packet_data, 6);

  for (int i = 0; i < lsn; ++i) {
    const size_t sample_offset = kPacketHeaderSize + static_cast<size_t>(i) * kSampleSize;
    const uint8_t quality = packet_data[sample_offset];
    const uint16_t distance = bytesToUint16(packet_data, sample_offset + kSampleDistanceOffset);
    checksum ^= static_cast<uint16_t>(distance ^ quality);
  }

  return checksum;
}

bool LidarProtocol::isChecksumValid(const std::vector<uint8_t> & packet_data)
{
  if (packet_data.size() < kPacketHeaderSize) {
    return false;
  }
  const uint16_t expected = bytesToUint16(packet_data, 8);
  return calculateChecksum(packet_data) == expected;
}

std::optional<LidarPacket> LidarProtocol::parsePacket(const std::vector<uint8_t> & packet_data)
{
  if (packet_data.size() < kPacketHeaderSize || packet_data[0] != 0xAA || packet_data[1] != 0x55) {
    return std::nullopt;
  }

  LidarPacket packet;
  packet.ct = packet_data[2];
  packet.lsn = packet_data[3];
  packet.is_scan_start = (packet.ct & 0x01) != 0;
  if (packet.lsn == 0 || packet_data.size() < expectedPacketSize(packet.lsn)) {
    return packet;
  }

  packet.fsa_raw = bytesToUint16(packet_data, 4);
  packet.lsa_raw = bytesToUint16(packet_data, 6);

  const double angle_start_deg = static_cast<double>(packet.fsa_raw >> 1) / 64.0;
  const double angle_end_deg = static_cast<double>(packet.lsa_raw >> 1) / 64.0;

  double diff_angle_deg = 0.0;
  if (packet.lsn > 1) {
    diff_angle_deg = angle_end_deg - angle_start_deg;
    if (diff_angle_deg < 0.0) {
      diff_angle_deg += 360.0;
    }
  }

  packet.points.reserve(packet.lsn);
  for (int i = 0; i < packet.lsn; ++i) {
    const size_t sample_offset = kPacketHeaderSize + static_cast<size_t>(i) * kSampleSize;
    const size_t distance_offset = sample_offset + kSampleDistanceOffset;
    if (distance_offset + 1 >= packet_data.size()) {
      break;
    }

    const uint8_t quality = packet_data[sample_offset];
    const uint16_t dist_raw = bytesToUint16(packet_data, distance_offset);
    const double distance_m = static_cast<double>(dist_raw) / 4.0 / 1000.0;
    const double distance_mm = static_cast<double>(dist_raw) / 4.0;
    if (distance_m <= 0.01) {
      continue;
    }

    double angle_deg = angle_start_deg;
    if (packet.lsn > 1) {
      angle_deg = (diff_angle_deg / (packet.lsn - 1)) * i + angle_start_deg;
    }

    double angle_correct_deg = 0.0;
    if (distance_mm != 0.0) {
      const double numerator =
        kAngleCorrectionOffsetMm * (kAngleCorrectionBaselineMm - distance_mm);
      const double denominator = kAngleCorrectionBaselineMm * distance_mm;
      angle_correct_deg = std::atan(numerator / denominator) * 180.0 / M_PI;
    }

    double final_angle_deg = std::fmod(angle_deg + angle_correct_deg, 360.0);
    if (final_angle_deg < 0.0) {
      final_angle_deg += 360.0;
    }

    LidarPoint point;
    point.angle_rad = M_PI * final_angle_deg / 180.0;
    point.distance_m = distance_m;
    point.quality = quality;
    point.x = distance_m * std::cos(point.angle_rad);
    point.y = distance_m * std::sin(point.angle_rad);
    packet.points.push_back(point);
  }

  return packet;
}

}  // namespace xlerobot_lidar_driver
