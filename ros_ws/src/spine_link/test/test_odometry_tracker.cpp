#include <gtest/gtest.h>

#include "spine_link/odometry_tracker.hpp"

using spine::OdometryTracker;
using Outcome = OdometryTracker::Outcome;

namespace
{
double ts(std::int64_t seq, double base = 50.0) {return base + static_cast<double>(seq) * 0.005;}
}  // namespace

TEST(OdometryTracker, InOrderAndGaps)
{
  OdometryTracker t;
  EXPECT_EQ(t.process(0, ts(0)).outcome, Outcome::kAccept);
  EXPECT_EQ(t.process(1, ts(1)).gap, 0);
  auto r = t.process(5, ts(5));
  EXPECT_EQ(r.outcome, Outcome::kAccept);
  EXPECT_EQ(r.gap, 3);
}

TEST(OdometryTracker, DuplicateAndReorderedAreStale)
{
  OdometryTracker t;
  t.process(10, ts(10));
  EXPECT_EQ(t.process(10, ts(10)).outcome, Outcome::kStale);
  EXPECT_EQ(t.process(8, ts(8)).outcome, Outcome::kStale);
  EXPECT_EQ(t.process(11, ts(11)).gap, 0);
}

TEST(OdometryTracker, ProgramRestartSeqBackTimeForward)
{
  OdometryTracker t;
  t.process(10, ts(10));
  EXPECT_EQ(t.process(0, ts(0, 90.0)).outcome, Outcome::kRestart);
  EXPECT_EQ(t.process(1, ts(1, 90.0)).outcome, Outcome::kAccept);
}

TEST(OdometryTracker, RebootSeqFarBack)
{
  OdometryTracker t;
  t.process(5000, ts(5000));
  EXPECT_EQ(t.process(0, ts(0, 20.0)).outcome, Outcome::kRestart);
}

TEST(OdometryTracker, RebootSeqForwardTimeBack)
{
  OdometryTracker t;
  t.process(10, ts(10));
  auto r = t.process(40, ts(40, 1.0));
  EXPECT_EQ(r.outcome, Outcome::kRestart);
  EXPECT_EQ(r.gap, 0) << "no gap across a restart";
}
