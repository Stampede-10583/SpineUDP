#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

#include "spine_link/pose_pipeline.hpp"

namespace p = spine::protocol;
using spine::PoseInput;
using spine::PosePipeline;
using Verdict = PosePipeline::Verdict;

namespace
{

constexpr double kNewestTrigger = 1000.0;

struct Fixture
{
  PosePipeline pipe{1.4, {}};
  std::array<std::uint8_t, p::kPoseSize> out{};

  Verdict run(const PoseInput & in, bool has_trigger = true, double newest = kNewestTrigger)
  {
    return pipe.process(in, has_trigger, newest, out);
  }
};

PoseInput good(double ts) {return {ts, 1.0, 2.0, 0.5, true};}

}  // namespace

TEST(PosePipeline, ValidPoseTimestampBytesEqualInputBitsExactly)
{
  const double awkward[] = {
    // Strictly increasing, so the order gate admits each one.
    999.9,
    std::bit_cast<double>(std::uint64_t{0x408F3FFFFFFFFFFE}),  // 2 ulp below 1000, low bits set
    std::nextafter(1000.0, 0.0),  // 1 ulp below 1000: every mantissa bit set
  };
  Fixture f;
  for (double ts : awkward) {
    ASSERT_EQ(f.run(good(ts)), Verdict::kSend);
    std::uint8_t expected[8];
    p::store_le_u64(expected, std::bit_cast<std::uint64_t>(ts));
    EXPECT_EQ(0, std::memcmp(f.out.data() + 8, expected, 8));
    auto decoded = p::decode_pose(f.out.data(), f.out.size());
    ASSERT_TRUE(decoded);
    EXPECT_EQ(std::bit_cast<std::uint64_t>(decoded->timestamp), std::bit_cast<std::uint64_t>(ts));
    EXPECT_EQ(decoded->x, 1.0);
    EXPECT_EQ(decoded->y, 2.0);
    EXPECT_EQ(decoded->theta, 0.5);
  }
}

TEST(PosePipeline, SeqCountsSentPosesOnly)
{
  Fixture f;
  ASSERT_EQ(f.run(good(999.0)), Verdict::kSend);
  EXPECT_EQ(p::load_le_i64(f.out.data()), 0);
  ASSERT_EQ(f.run({999.1, 0, 0, 0, false}), Verdict::kNoFix);
  ASSERT_EQ(f.run(good(999.2)), Verdict::kSend);
  EXPECT_EQ(p::load_le_i64(f.out.data()), 1);
}

TEST(PosePipeline, NoFixIsWithheld)
{
  Fixture f;
  EXPECT_EQ(f.run({999.0, 1, 2, 3, false}), Verdict::kNoFix);
  EXPECT_EQ(f.run({999.0, std::nan(""), 2, 3, true}), Verdict::kNoFix);
  EXPECT_EQ(f.run({999.0, 1, std::numeric_limits<double>::infinity(), 3, true}), Verdict::kNoFix);
  EXPECT_FALSE(f.pipe.gate().has_last()) << "withheld poses must not move the order gate";
}

TEST(PosePipeline, NaNTimestampIsWithheld)
{
  Fixture f;
  EXPECT_EQ(f.run(good(std::nan(""))), Verdict::kBadTimestamp);
}

TEST(PosePipeline, StaleIsWithheldAgainstNewestTriggerOnly)
{
  Fixture f;
  EXPECT_EQ(f.run(good(kNewestTrigger - 1.5)), Verdict::kStale);
  EXPECT_EQ(f.run(good(-std::numeric_limits<double>::infinity())), Verdict::kStale);
  EXPECT_EQ(f.run(good(kNewestTrigger - 1.4)), Verdict::kSend) << "exactly max_age is kept";
}

TEST(PosePipeline, NoTriggerYetIsWithheldAsStale)
{
  Fixture f;
  EXPECT_EQ(f.run(good(999.0), /*has_trigger=*/false), Verdict::kStale);
}

TEST(PosePipeline, OutOfOrderAndDuplicateAreWithheld)
{
  Fixture f;
  ASSERT_EQ(f.run(good(999.5)), Verdict::kSend);
  EXPECT_EQ(f.run(good(999.4)), Verdict::kOutOfOrder);
  EXPECT_EQ(f.run(good(999.5)), Verdict::kOutOfOrder);
  EXPECT_EQ(f.run(good(999.6)), Verdict::kSend);
}

TEST(PosePipeline, RebootReopensTheGate)
{
  Fixture f;
  ASSERT_EQ(f.run(good(999.5)), Verdict::kSend);
  // New run after a reboot: RIO time restarted near 0.
  f.pipe.on_rio_restart(20.1);
  EXPECT_EQ(f.run(good(20.0), true, 20.1), Verdict::kSend);
}

TEST(PosePipeline, ProgramRestartKeepsTheGateSoNothingOlderIsEverSent)
{
  Fixture f;
  ASSERT_EQ(f.run(good(999.5)), Verdict::kSend);
  // Robot program restarted; the RIO clock kept running, so the new run's triggers are later.
  f.pipe.on_rio_restart(1000.2);
  // A late pose from the old run must still be withheld, even though it isn't stale yet.
  EXPECT_EQ(f.run(good(999.4), true, 1000.2), Verdict::kOutOfOrder);
  EXPECT_EQ(f.run(good(1000.2), true, 1000.2), Verdict::kSend);
}

TEST(PosePipeline, FieldFromMapTransformsPoseButNeverTheTimestamp)
{
  PosePipeline pipe(1.4, {10.0, -2.0, std::numbers::pi / 2});
  std::array<std::uint8_t, p::kPoseSize> out{};
  const double ts = 999.123456789;
  ASSERT_EQ(pipe.process({ts, 1.0, 0.0, 3.0, true}, true, kNewestTrigger, out), Verdict::kSend);
  auto d = p::decode_pose(out.data(), out.size());
  ASSERT_TRUE(d);
  EXPECT_NEAR(d->x, 10.0, 1e-12);
  EXPECT_NEAR(d->y, -1.0, 1e-12);
  EXPECT_NEAR(d->theta, 3.0 + std::numbers::pi / 2 - 2 * std::numbers::pi, 1e-12) << "wrapped";
  EXPECT_EQ(std::bit_cast<std::uint64_t>(d->timestamp), std::bit_cast<std::uint64_t>(ts));
}
