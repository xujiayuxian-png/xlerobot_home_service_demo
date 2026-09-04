#include <gtest/gtest.h>

#include <cmath>

#include "xlerobot_hardware/calibrated_servo_codec.hpp"

namespace xlerobot_hardware
{

TEST(CalibratedServoCodecTest, RoundTripsBothDirections)
{
  for (const int direction : {-1, 1}) {
    CalibratedServoCodec codec({2048, direction, 1000, 3100, -1.5, 1.5});
    ASSERT_TRUE(codec.valid());
    const auto raw = codec.encode(0.7);
    ASSERT_TRUE(raw.has_value());
    const auto decoded = codec.decode(*raw);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_NEAR(*decoded, 0.7, 2.0 * M_PI / 4096.0);
  }
}

TEST(CalibratedServoCodecTest, RejectsUnsafeInputs)
{
  CalibratedServoCodec codec({2048, 1, 1000, 3100, -1.0, 1.0});
  EXPECT_FALSE(codec.encode(std::nan("")));
  EXPECT_FALSE(codec.encode(1.01));
  EXPECT_FALSE(codec.decode(999));
  EXPECT_FALSE(CalibratedServoCodec({2048, 0, 0, 4095, -1.0, 1.0}).valid());
}

TEST(CalibratedServoCodecTest, SaturatesValidJointCommandAtCalibratedRawBoundary)
{
  CalibratedServoCodec codec({2048, 1, 1500, 2500, -1.0, 1.0});

  ASSERT_TRUE(codec.encode(-1.0));
  ASSERT_TRUE(codec.encode(1.0));
  EXPECT_EQ(*codec.encode(-1.0), 1500);
  EXPECT_EQ(*codec.encode(1.0), 2500);
}

TEST(CalibratedServoCodecTest, ObservesOutsideCommandEnvelopeWithoutMakingItCommandable)
{
  CalibratedServoCodec codec({2048, 1, 1000, 3100, -1.0, 1.0});
  ASSERT_TRUE(codec.decode_observation(500).has_value());
  EXPECT_FALSE(codec.decode(500));
  EXPECT_FALSE(codec.encode(*codec.decode_observation(500)));
  EXPECT_FALSE(codec.decode_observation(-1));
  EXPECT_FALSE(codec.decode_observation(4096));
}

}  // namespace xlerobot_hardware
