#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include "xlerobot_lidar_driver/lidar_packet_parser.hpp"
#include "xlerobot_lidar_driver/lidar_protocol.hpp"
#include "xlerobot_lidar_driver/scan_accumulator.hpp"
#include "xlerobot_lidar_driver/scan_builder.hpp"
#include "xlerobot_lidar_driver/scan_filter.hpp"
#include "xlerobot_lidar_driver/serial_port.hpp"

namespace xlerobot_lidar_driver
{
namespace
{

std::vector<uint8_t> knownPacket()
{
  return {
    0xAA, 0x55, 0x01, 0x02, 0x01, 0x00, 0x01, 0x2D,
    0x7F, 0x6A, 0x64, 0xA0, 0x0F, 0x50, 0x40, 0x1F};
}

LidarPoint makePoint(double angle_rad, double distance_m, uint8_t quality = 1)
{
  LidarPoint point;
  point.angle_rad = angle_rad;
  point.distance_m = distance_m;
  point.quality = quality;
  point.x = distance_m * std::cos(angle_rad);
  point.y = distance_m * std::sin(angle_rad);
  return point;
}

}  // namespace

TEST(LidarProtocolTest, ValidatesChecksumAndPacketLayout)
{
  const auto packet_data = knownPacket();

  EXPECT_EQ(LidarProtocol::expectedPacketSize(2), packet_data.size());
  EXPECT_EQ(LidarProtocol::calculateChecksum(packet_data), 0x6A7F);
  EXPECT_TRUE(LidarProtocol::isChecksumValid(packet_data));

  const auto packet = LidarProtocol::parsePacket(packet_data);
  ASSERT_TRUE(packet.has_value());
  EXPECT_TRUE(packet->is_scan_start);
  EXPECT_EQ(packet->ct, 0x01);
  EXPECT_EQ(packet->lsn, 2);
  ASSERT_EQ(packet->points.size(), 2U);
  EXPECT_DOUBLE_EQ(packet->points[0].distance_m, 1.0);
  EXPECT_DOUBLE_EQ(packet->points[1].distance_m, 2.0);
  EXPECT_EQ(packet->points[0].quality, 0x64);
  EXPECT_EQ(packet->points[1].quality, 0x50);
}

TEST(LidarProtocolTest, RejectsShortBuffersBeforeChecksumMath)
{
  const std::vector<uint8_t> too_short{0xAA, 0x55, 0x01};

  EXPECT_THROW(
    static_cast<void>(LidarProtocol::bytesToUint16(too_short, 2)),
    std::out_of_range);
  EXPECT_EQ(LidarProtocol::calculateChecksum(too_short), 0U);
  EXPECT_FALSE(LidarProtocol::isChecksumValid(too_short));

  auto truncated_packet = knownPacket();
  truncated_packet.pop_back();
  EXPECT_EQ(LidarProtocol::calculateChecksum(truncated_packet), 0U);
  EXPECT_FALSE(LidarProtocol::isChecksumValid(truncated_packet));
}

TEST(LidarPacketParserTest, EmitsCompletePacketsAndCountsChecksumFailures)
{
  LidarPacketParser parser(true);
  std::optional<LidarPacket> parsed;

  for (const uint8_t byte : knownPacket()) {
    parsed = parser.processByte(byte);
  }

  ASSERT_TRUE(parsed.has_value());
  EXPECT_TRUE(parsed->is_scan_start);
  EXPECT_EQ(parser.invalidChecksumCount(), 0U);

  auto bad_packet = knownPacket();
  bad_packet[8] ^= 0x01;
  for (const uint8_t byte : bad_packet) {
    parsed = parser.processByte(byte);
  }

  EXPECT_FALSE(parsed.has_value());
  EXPECT_EQ(parser.invalidChecksumCount(), 1U);
  EXPECT_NE(parser.lastExpectedChecksum(), parser.lastActualChecksum());
}

TEST(ScanAccumulatorTest, CtBitStartsANewScan)
{
  ScanAccumulator accumulator;
  LidarPacket first;
  first.points.push_back(makePoint(0.2, 1.0));
  EXPECT_TRUE(accumulator.addPacket(first).empty());

  LidarPacket start;
  start.is_scan_start = true;
  start.points.push_back(makePoint(0.3, 1.2));
  const auto completed = accumulator.addPacket(start);

  ASSERT_EQ(completed.size(), 1U);
  ASSERT_EQ(completed[0].points.size(), 1U);
  EXPECT_DOUBLE_EQ(completed[0].points[0].distance_m, 1.0);
}

TEST(ScanAccumulatorTest, CtBitDoesNotEmitEmptyScan)
{
  ScanAccumulator accumulator;
  LidarPacket start;
  start.is_scan_start = true;
  start.points.push_back(makePoint(0.3, 1.2));

  const auto completed = accumulator.addPacket(start);

  EXPECT_TRUE(completed.empty());
}

TEST(ScanAccumulatorTest, AngleWrapFallbackStartsANewScan)
{
  ScanAccumulator accumulator;
  LidarPacket before_wrap;
  before_wrap.points.push_back(makePoint(5.9, 1.0));
  EXPECT_TRUE(accumulator.addPacket(before_wrap).empty());

  LidarPacket after_wrap;
  after_wrap.points.push_back(makePoint(0.1, 1.1));
  const auto completed = accumulator.addPacket(after_wrap);

  ASSERT_EQ(completed.size(), 1U);
  ASSERT_EQ(completed[0].points.size(), 1U);
  EXPECT_NEAR(completed[0].points[0].angle_rad, 5.9, 1e-9);
}

TEST(ScanAccumulatorTest, CtModeIgnoresFirstAngleWrapAfterBoundary)
{
  ScanAccumulator accumulator;
  LidarPacket scan_start;
  scan_start.is_scan_start = true;
  scan_start.points.push_back(makePoint(5.9, 1.0));
  EXPECT_TRUE(accumulator.addPacket(scan_start).empty());

  LidarPacket normal_wrap_after_ct;
  normal_wrap_after_ct.points.push_back(makePoint(0.1, 1.1));
  const auto completed = accumulator.addPacket(normal_wrap_after_ct);

  EXPECT_TRUE(completed.empty());
}

TEST(ScanAccumulatorTest, AngleWrapFallbackWorksWhenNextCtBoundaryIsMissing)
{
  ScanAccumulator accumulator;
  LidarPacket scan_start;
  scan_start.is_scan_start = true;
  scan_start.points.push_back(makePoint(5.9, 1.0));
  EXPECT_TRUE(accumulator.addPacket(scan_start).empty());

  LidarPacket normal_wrap_after_ct;
  normal_wrap_after_ct.points.push_back(makePoint(0.1, 1.1));
  EXPECT_TRUE(accumulator.addPacket(normal_wrap_after_ct).empty());

  LidarPacket before_next_wrap;
  before_next_wrap.points.push_back(makePoint(5.9, 1.2));
  EXPECT_TRUE(accumulator.addPacket(before_next_wrap).empty());

  LidarPacket missing_ct_wrap;
  missing_ct_wrap.points.push_back(makePoint(0.1, 1.3));
  const auto completed = accumulator.addPacket(missing_ct_wrap);

  ASSERT_EQ(completed.size(), 1U);
  ASSERT_EQ(completed[0].points.size(), 3U);
  EXPECT_NEAR(completed[0].points[0].angle_rad, 5.9, 1e-9);
}

TEST(ScanFilterTest, RadiusFilterRemovesIsolatedPoints)
{
  RadiusFilterConfig radius;
  radius.enabled = true;
  radius.radius_m = 0.2;
  radius.min_neighbors = 1;
  ScanFilter filter(radius, FixedMaskConfig{}, AutoBodyPostsMaskConfig{});

  std::vector<LidarPoint> points{
    makePoint(0.0, 1.0),
    makePoint(0.01, 1.02),
    makePoint(2.0, 3.0),
  };
  filter.removeOutliers(points);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_NEAR(points[0].distance_m, 1.0, 1e-9);
  EXPECT_NEAR(points[1].distance_m, 1.02, 1e-9);
}

TEST(ScanFilterTest, RadiusFilterDoesNotCountPointAsOwnNeighbor)
{
  RadiusFilterConfig radius;
  radius.enabled = true;
  radius.radius_m = 0.2;
  radius.min_neighbors = 2;
  ScanFilter filter(radius, FixedMaskConfig{}, AutoBodyPostsMaskConfig{});

  std::vector<LidarPoint> points{
    makePoint(0.0, 1.0),
    makePoint(0.01, 1.02),
    makePoint(2.0, 3.0),
  };
  filter.removeOutliers(points);

  EXPECT_TRUE(points.empty());
}

TEST(ScanFilterTest, FixedMaskHandlesNormalAndWrappedRanges)
{
  FixedMaskConfig fixed;
  fixed.enabled = true;
  fixed.angle_ranges_deg = {80.0, 100.0, 350.0, 10.0};
  fixed.max_range_m = 0.0;
  ScanFilter filter(RadiusFilterConfig{}, fixed, AutoBodyPostsMaskConfig{});

  EXPECT_TRUE(filter.isFixedMasked(M_PI / 2.0, 1.0));
  EXPECT_TRUE(filter.isFixedMasked(0.0, 1.0));
  EXPECT_FALSE(filter.isFixedMasked(M_PI, 1.0));
}

TEST(ScanBuilderTest, KeepsNearestRangeAndIntensity)
{
  RadiusFilterConfig radius;
  radius.enabled = false;
  ScanBuilder builder(ScanBuilderOptions{},
    ScanFilter(radius, FixedMaskConfig{}, AutoBodyPostsMaskConfig{}));
  builtin_interfaces::msg::Time stamp;

  builder.addPoint(makePoint(0.0, 1.0, 10));
  builder.addPoint(makePoint(0.0, 0.5, 9));
  const auto result = builder.build(stamp);

  ASSERT_EQ(result.scan.ranges.size(), 720U);
  EXPECT_FLOAT_EQ(result.scan.ranges[0], 0.5f);
  EXPECT_FLOAT_EQ(result.scan.intensities[0], 9.0f);
  EXPECT_TRUE(builder.empty());
}

TEST(ScanBuilderTest, UsesMeasuredScanTimeWhenProvided)
{
  RadiusFilterConfig radius;
  radius.enabled = false;
  ScanBuilder builder(ScanBuilderOptions{},
    ScanFilter(radius, FixedMaskConfig{}, AutoBodyPostsMaskConfig{}));
  builtin_interfaces::msg::Time stamp;

  builder.addPoint(makePoint(0.0, 1.0, 10));
  const auto result = builder.build(stamp, 0.167);

  EXPECT_FLOAT_EQ(result.scan.scan_time, 0.167f);
  EXPECT_FLOAT_EQ(result.scan.time_increment, 0.167f / 720.0f);
}

TEST(ScanBuilderTest, AppliesFixedMaskToPublishedAngle)
{
  RadiusFilterConfig radius;
  radius.enabled = false;
  FixedMaskConfig fixed;
  fixed.enabled = true;
  fixed.angle_ranges_deg = {350.0, 10.0};
  fixed.max_range_m = 0.0;
  ScanBuilder builder(ScanBuilderOptions{}, ScanFilter(radius, fixed, AutoBodyPostsMaskConfig{}));
  builtin_interfaces::msg::Time stamp;

  builder.addPoint(makePoint(0.0, 1.0, 10));
  const auto result = builder.build(stamp);

  EXPECT_TRUE(std::isinf(result.scan.ranges[0]));
}

TEST(ScanFilterTest, AutoBodyPostsMaskHandlesWrappedClusters)
{
  AutoBodyPostsMaskConfig auto_mask;
  auto_mask.enabled = true;
  auto_mask.min_range_m = 0.12;
  auto_mask.max_range_m = 0.35;
  auto_mask.padding_deg = 1.0;
  auto_mask.cluster_gap_deg = 2.0;
  auto_mask.max_clusters = 1;
  auto_mask.min_cluster_points = 1;
  ScanFilter filter(RadiusFilterConfig{}, FixedMaskConfig{}, auto_mask);

  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 2.0 * M_PI / 720.0;
  scan.ranges.assign(720, std::numeric_limits<float>::infinity());
  scan.ranges[719] = 0.22f;
  scan.ranges[0] = 0.20f;

  const auto clusters = filter.applyAutoBodyPostsMask(scan);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_TRUE(std::isinf(scan.ranges[719]));
  EXPECT_TRUE(std::isinf(scan.ranges[0]));
  EXPECT_TRUE(std::isinf(scan.ranges[1]));
}

TEST(ScanFilterTest, AutoBodyPostsMaskIgnoresRangesOutsideConfiguredBand)
{
  AutoBodyPostsMaskConfig auto_mask;
  auto_mask.enabled = true;
  auto_mask.min_range_m = 0.12;
  auto_mask.max_range_m = 0.35;
  auto_mask.padding_deg = 1.0;
  auto_mask.cluster_gap_deg = 2.0;
  auto_mask.max_clusters = 4;
  auto_mask.min_cluster_points = 1;
  ScanFilter filter(RadiusFilterConfig{}, FixedMaskConfig{}, auto_mask);

  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 2.0 * M_PI / 720.0;
  scan.ranges.assign(720, std::numeric_limits<float>::infinity());
  scan.ranges[100] = 0.10f;
  scan.ranges[200] = 0.40f;
  scan.ranges[300] = 0.20f;

  const auto clusters = filter.applyAutoBodyPostsMask(scan);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_FLOAT_EQ(scan.ranges[100], 0.10f);
  EXPECT_FLOAT_EQ(scan.ranges[200], 0.40f);
  EXPECT_TRUE(std::isinf(scan.ranges[300]));
}

TEST(SerialPortTest, WritesBytesAndStopCommandToPseudoTerminal)
{
  const int master_fd = ::posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_NE(master_fd, -1);
  ASSERT_EQ(::grantpt(master_fd), 0);
  ASSERT_EQ(::unlockpt(master_fd), 0);
  char * slave_name = ::ptsname(master_fd);
  ASSERT_NE(slave_name, nullptr);

  SerialPort port;
  ASSERT_TRUE(port.openPort(std::string(slave_name), 150000));
  ASSERT_TRUE(port.writeBytes({0x01, 0x02, 0x03}));

  uint8_t buffer[8] = {};
  ssize_t bytes_read = ::read(master_fd, buffer, 3);
  ASSERT_EQ(bytes_read, 3);
  EXPECT_EQ(buffer[0], 0x01);
  EXPECT_EQ(buffer[1], 0x02);
  EXPECT_EQ(buffer[2], 0x03);

  port.sendStopCommand();
  bytes_read = ::read(master_fd, buffer, 2);
  ASSERT_EQ(bytes_read, 2);
  EXPECT_EQ(buffer[0], 0xA5);
  EXPECT_EQ(buffer[1], 0x65);

  port.closePort();
  ::close(master_fd);
}

}  // namespace xlerobot_lidar_driver
