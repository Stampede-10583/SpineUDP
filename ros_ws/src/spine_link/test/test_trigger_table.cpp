#include <gtest/gtest.h>

#include <cmath>

#include "spine_link/trigger_table.hpp"

namespace p = spine::protocol;
using spine::TriggerTable;
using Outcome = TriggerTable::Outcome;

namespace
{

// RIO run model: trigger s fires at base + s / 60 s.
double ts_of(std::int64_t s, double base) {return base + static_cast<double>(s) / 60.0;}

// The packet the RIO sends right after recording trigger `newest` (0-based) in a run.
p::TriggerPacket packet(std::int64_t newest, double base = 100.0, std::int64_t packet_seq = 0)
{
  p::TriggerPacket pk;
  pk.packet_seq = packet_seq;
  for (std::size_t i = 0; i < p::kTrigEntryCount; ++i) {
    const std::int64_t s = newest - static_cast<std::int64_t>(i);
    pk.entries[i] = s >= 0 ? p::TriggerEntry{s, ts_of(s, base)} : p::TriggerEntry{-1, 0.0};
  }
  return pk;
}

std::vector<std::int64_t> fresh_seqs(const TriggerTable::Result & r)
{
  std::vector<std::int64_t> out;
  for (std::size_t i = 0; i < r.fresh_count; ++i) {
    out.push_back(r.fresh[i].trigger_seq);
  }
  return out;
}

using Seqs = std::vector<std::int64_t>;

}  // namespace

TEST(TriggerTable, FirstPacketHasFourSentinels)
{
  TriggerTable t;
  EXPECT_FALSE(t.has_trigger());
  auto r = t.process(packet(0));
  EXPECT_EQ(r.outcome, Outcome::kNew);
  EXPECT_EQ(fresh_seqs(r), Seqs{0});
  EXPECT_EQ(r.fresh[0].timestamp, ts_of(0, 100.0));
  EXPECT_FALSE(r.restarted);
  EXPECT_EQ(r.missed, 0);
  EXPECT_TRUE(t.has_trigger());
  EXPECT_EQ(t.newest_timestamp(), ts_of(0, 100.0));
}

TEST(TriggerTable, SteadyStateOneNewPerPacket)
{
  TriggerTable t;
  for (std::int64_t s = 0; s < 20; ++s) {
    auto r = t.process(packet(s));
    ASSERT_EQ(fresh_seqs(r), Seqs{s});
    EXPECT_EQ(r.missed, 0);
  }
  EXPECT_EQ(t.last_seen_seq(), 19);
}

TEST(TriggerTable, CoalescedPacketsYieldAllNewOldestFirst)
{
  TriggerTable t;
  t.process(packet(3));  // 0..3
  auto r2 = t.process(packet(5));
  EXPECT_EQ(fresh_seqs(r2), (Seqs{4, 5}));
  auto r3 = t.process(packet(8));
  EXPECT_EQ(fresh_seqs(r3), (Seqs{6, 7, 8}));
  EXPECT_EQ(r3.missed, 0);
  EXPECT_EQ(t.newest_timestamp(), ts_of(8, 100.0));
}

TEST(TriggerTable, DuplicateAndReorderedPacketsYieldNothing)
{
  TriggerTable t;
  t.process(packet(10));
  EXPECT_EQ(t.process(packet(10)).outcome, Outcome::kNothingNew);  // duplicate
  auto r = t.process(packet(8));  // older packet arriving late
  EXPECT_EQ(r.outcome, Outcome::kNothingNew);
  EXPECT_FALSE(r.restarted);
  EXPECT_EQ(t.last_seen_seq(), 10);
  EXPECT_EQ(fresh_seqs(t.process(packet(11))), Seqs{11});
}

TEST(TriggerTable, FiveOrMoreLostPacketsCountTheGap)
{
  TriggerTable t;
  t.process(packet(10));
  // Packets for 11..16 lost; packet 17 carries 13..17, so 11 and 12 are gone for good.
  auto r = t.process(packet(17));
  EXPECT_EQ(fresh_seqs(r), (Seqs{13, 14, 15, 16, 17}));
  EXPECT_EQ(r.missed, 2);
  EXPECT_FALSE(r.restarted);
}

TEST(TriggerTable, FourLostPacketsLoseNothing)
{
  TriggerTable t;
  t.process(packet(10));
  auto r = t.process(packet(15));  // 11..14 lost, packet 15 carries 11..15
  EXPECT_EQ(fresh_seqs(r), (Seqs{11, 12, 13, 14, 15}));
  EXPECT_EQ(r.missed, 0);
}

TEST(TriggerTable, RestartBySeqJumpingBack)
{
  TriggerTable t;
  t.process(packet(500, 100.0));
  // New run on a rebooted RIO: seq back to 0, clock back near 0.
  auto r = t.process(packet(0, 20.0));
  EXPECT_TRUE(r.restarted);
  EXPECT_EQ(fresh_seqs(r), Seqs{0});
  EXPECT_EQ(r.missed, 0);
  EXPECT_EQ(t.newest_timestamp(), ts_of(0, 20.0));
}

TEST(TriggerTable, RestartBySeqBackButTimeForward)
{
  TriggerTable t;
  t.process(packet(3, 100.0));  // short first run: seq 3 only, so "seq < last - 5" can't fire
  // Robot program restarted (RIO clock keeps running): seq 1 of the new run, at a later time.
  auto r = t.process(packet(1, 200.0));
  EXPECT_TRUE(r.restarted);
  EXPECT_EQ(fresh_seqs(r), (Seqs{0, 1}));
}

TEST(TriggerTable, RestartBySeqForwardButTimeBack)
{
  TriggerTable t;
  t.process(packet(3, 100.0));
  // RIO rebooted and we missed its first packets: seq already past 3, clock restarted near 0.
  auto r = t.process(packet(9, 20.0));
  EXPECT_TRUE(r.restarted);
  EXPECT_EQ(fresh_seqs(r), (Seqs{5, 6, 7, 8, 9}));
  EXPECT_EQ(r.missed, 0) << "no gap is counted across a restart";
}

TEST(TriggerTable, ReorderedPacketIsNotMistakenForRestart)
{
  TriggerTable t;
  t.process(packet(20));
  auto r = t.process(packet(16));  // 4 behind, older time: plain reordering
  EXPECT_FALSE(r.restarted);
  EXPECT_EQ(r.outcome, Outcome::kNothingNew);
}

TEST(TriggerTable, InconsistentPacketIsRejectedAndChangesNothing)
{
  TriggerTable t;
  t.process(packet(10));

  auto gap = packet(20);
  gap.entries[2].trigger_seq = 17;  // not consecutive
  EXPECT_EQ(t.process(gap).outcome, Outcome::kDecodeError);

  auto sentinel_mid = packet(20);
  sentinel_mid.entries[1] = {-1, 0.0};  // real entry after a sentinel
  EXPECT_EQ(t.process(sentinel_mid).outcome, Outcome::kDecodeError);

  auto time_backwards = packet(20);
  time_backwards.entries[3].timestamp = 1e9;  // older trigger stamped later
  EXPECT_EQ(t.process(time_backwards).outcome, Outcome::kDecodeError);

  auto nan_ts = packet(20);
  nan_ts.entries[0].timestamp = std::nan("");
  EXPECT_EQ(t.process(nan_ts).outcome, Outcome::kDecodeError);

  auto bad_sentinel = packet(1);
  bad_sentinel.entries[3].timestamp = 5.0;  // sentinel with a nonzero timestamp
  EXPECT_EQ(t.process(bad_sentinel).outcome, Outcome::kDecodeError);

  EXPECT_EQ(t.last_seen_seq(), 10);
  EXPECT_EQ(fresh_seqs(t.process(packet(11))), Seqs{11});
}

TEST(TriggerTable, AllSentinelPacketYieldsNothing)
{
  TriggerTable t;
  auto r = t.process(packet(-1));
  EXPECT_EQ(r.outcome, Outcome::kNothingNew);
  EXPECT_FALSE(t.has_trigger());
}
