#include <gtest/gtest.h>

#include <cmath>

#include "spine_link/order_gate.hpp"

TEST(OrderGate, AdmitsOnlyStrictlyNewer)
{
  spine::OrderGate g;
  EXPECT_TRUE(g.admit(10.0));
  EXPECT_TRUE(g.admit(10.5));
  EXPECT_FALSE(g.admit(10.25)) << "older than the last sent";
  EXPECT_FALSE(g.admit(10.5)) << "equal to the last sent (duplicate)";
  EXPECT_TRUE(g.admit(std::nextafter(10.5, 11.0))) << "one ulp newer is newer";
  EXPECT_DOUBLE_EQ(g.last(), std::nextafter(10.5, 11.0));
}

TEST(OrderGate, RejectedPoseDoesNotMoveTheGate)
{
  spine::OrderGate g;
  g.admit(10.0);
  g.admit(5.0);
  EXPECT_TRUE(g.admit(6.0) == false);
  EXPECT_EQ(g.last(), 10.0);
}

TEST(OrderGate, NeverAdmitsNaN)
{
  spine::OrderGate g;
  EXPECT_FALSE(g.admit(std::nan("")));
  EXPECT_FALSE(g.has_last());
  g.admit(1.0);
  EXPECT_FALSE(g.admit(std::nan("")));
}

TEST(OrderGate, ResetAcceptsOlderTimestampsAgain)
{
  spine::OrderGate g;
  g.admit(100.0);
  EXPECT_FALSE(g.admit(20.0));
  g.reset();
  EXPECT_TRUE(g.admit(20.0));
}
