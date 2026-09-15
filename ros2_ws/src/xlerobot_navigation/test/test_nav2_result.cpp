#include <gtest/gtest.h>

#include "xlerobot_navigation/nav2_result.hpp"

namespace xlerobot_navigation
{
namespace
{
struct HumbleResult {};
struct JazzyResult
{
  static constexpr uint16_t COLLISION_AHEAD = 3;
  static constexpr uint16_t TIMEOUT = 4;
  uint16_t error_code = 0;
  std::string error_msg;
};

TEST(Nav2ResultTest, HumbleDoesNotInventCollisionOrTimeoutDetails)
{
  const HumbleResult result;
  EXPECT_EQ(Nav2Result<HumbleResult>::error_code(result), 0);
  EXPECT_FALSE(Nav2Result<HumbleResult>::collision(result));
  EXPECT_FALSE(Nav2Result<HumbleResult>::timeout(result));
  EXPECT_TRUE(Nav2Result<HumbleResult>::detail(result).empty());
}

TEST(Nav2ResultTest, JazzyRetainsStructuredFailures)
{
  JazzyResult result{JazzyResult::COLLISION_AHEAD, "obstacle behind robot"};
  EXPECT_EQ(Nav2Result<JazzyResult>::error_code(result), JazzyResult::COLLISION_AHEAD);
  EXPECT_TRUE(Nav2Result<JazzyResult>::collision(result));
  EXPECT_FALSE(Nav2Result<JazzyResult>::timeout(result));
  EXPECT_EQ(Nav2Result<JazzyResult>::detail(result), result.error_msg);
  result.error_code = JazzyResult::TIMEOUT;
  EXPECT_TRUE(Nav2Result<JazzyResult>::timeout(result));
}
}  // namespace
}  // namespace xlerobot_navigation
