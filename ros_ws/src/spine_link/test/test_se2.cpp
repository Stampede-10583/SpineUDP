#include <gtest/gtest.h>

#include <numbers>

#include "spine_link/se2.hpp"

using spine::Pose2;

TEST(Se2, RelativeThenComposeRoundTrips)
{
  const Pose2 a{1.0, -2.0, 2.5};
  const Pose2 b{-3.0, 0.5, -2.9};
  const Pose2 r = spine::compose(a, spine::relative(a, b));
  EXPECT_NEAR(r.x, b.x, 1e-12);
  EXPECT_NEAR(r.y, b.y, 1e-12);
  EXPECT_NEAR(r.theta, b.theta, 1e-12);
}

TEST(Se2, ComposeRotatesTheChildOffset)
{
  const Pose2 r = spine::compose({2.0, 4.0, std::numbers::pi / 2}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(r.x, 2.0, 1e-12);
  EXPECT_NEAR(r.y, 5.0, 1e-12);
  EXPECT_NEAR(r.theta, std::numbers::pi / 2, 1e-12);
}
