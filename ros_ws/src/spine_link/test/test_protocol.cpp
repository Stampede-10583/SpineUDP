#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "spine_link/protocol.hpp"
#include "vectors.hpp"

namespace p = spine::protocol;
using spine::test::as_f64;
using spine::test::as_i64;

namespace
{

std::uint64_t bits(double d) {return std::bit_cast<std::uint64_t>(d);}

void expect_f64(const spine::test::Vector & v, const std::string & key, double actual)
{
  EXPECT_EQ(bits(as_f64(v.fields.at(key))), bits(actual)) << v.stream << " " << v.name << " " << key;
}

void expect_i64(const spine::test::Vector & v, const std::string & key, std::int64_t actual)
{
  EXPECT_EQ(as_i64(v.fields.at(key)), actual) << v.stream << " " << v.name << " " << key;
}

}  // namespace

TEST(Protocol, GoldenVectorsDecodeAndReencodeBitIdentically)
{
  const auto vectors = spine::test::load_vectors(SPINE_VECTORS_FILE);
  int odom = 0, pose = 0, trig = 0;
  for (const auto & v : vectors) {
    SCOPED_TRACE(v.stream + " " + v.name);
    std::vector<std::uint8_t> out(v.expect.size(), 0xAA);
    if (v.stream == "odom") {
      ++odom;
      ASSERT_EQ(v.fields.size(), 8u);
      auto d = p::decode_odometry(v.expect.data(), v.expect.size());
      ASSERT_TRUE(d);
      expect_i64(v, "seq", d->seq);
      expect_f64(v, "timestamp", d->timestamp);
      expect_f64(v, "x", d->x);
      expect_f64(v, "y", d->y);
      expect_f64(v, "theta", d->theta);
      expect_f64(v, "vx", d->vx);
      expect_f64(v, "vy", d->vy);
      expect_f64(v, "omega", d->omega);
      p::OdometryPacket e{as_i64(v.fields.at("seq")), as_f64(v.fields.at("timestamp")),
        as_f64(v.fields.at("x")), as_f64(v.fields.at("y")), as_f64(v.fields.at("theta")),
        as_f64(v.fields.at("vx")), as_f64(v.fields.at("vy")), as_f64(v.fields.at("omega"))};
      p::encode_odometry(out.data(), e);
    } else if (v.stream == "pose") {
      ++pose;
      ASSERT_EQ(v.fields.size(), 5u);
      auto d = p::decode_pose(v.expect.data(), v.expect.size());
      ASSERT_TRUE(d);
      expect_i64(v, "seq", d->seq);
      expect_f64(v, "timestamp", d->timestamp);
      expect_f64(v, "x", d->x);
      expect_f64(v, "y", d->y);
      expect_f64(v, "theta", d->theta);
      p::PosePacket e{as_i64(v.fields.at("seq")), as_f64(v.fields.at("timestamp")),
        as_f64(v.fields.at("x")), as_f64(v.fields.at("y")), as_f64(v.fields.at("theta"))};
      p::encode_pose(out.data(), e);
    } else if (v.stream == "trig") {
      ++trig;
      ASSERT_EQ(v.fields.size(), 11u);
      auto d = p::decode_trigger(v.expect.data(), v.expect.size());
      ASSERT_TRUE(d);
      expect_i64(v, "packetSeq", d->packet_seq);
      p::TriggerPacket e;
      e.packet_seq = as_i64(v.fields.at("packetSeq"));
      for (std::size_t i = 0; i < p::kTrigEntryCount; ++i) {
        const auto si = "triggerSeq[" + std::to_string(i) + "]";
        const auto ti = "timestamp[" + std::to_string(i) + "]";
        expect_i64(v, si, d->entries[i].trigger_seq);
        expect_f64(v, ti, d->entries[i].timestamp);
        e.entries[i] = {as_i64(v.fields.at(si)), as_f64(v.fields.at(ti))};
      }
      p::encode_trigger(out.data(), e);
    } else {
      FAIL() << "unknown stream " << v.stream;
    }
    EXPECT_EQ(out, v.expect) << "re-encode differs";
  }
  EXPECT_GT(odom, 0);
  EXPECT_GT(pose, 0);
  EXPECT_GT(trig, 0);
}

// A timestamp read out of Stream 3 and written into Stream 2 must come back as the same 8 bytes,
// for every double, including the awkward ones.
TEST(Protocol, TimestampEchoIsBitExact)
{
  const double cases[] = {
    0.0, -0.0, 12.345678901234567,
    std::numeric_limits<double>::denorm_min(),
    std::numeric_limits<double>::min(),
    std::numeric_limits<double>::max(),
    std::bit_cast<double>(std::uint64_t{0x405FFFFFFFFFFFFF}),  // every mantissa bit set
    std::bit_cast<double>(std::uint64_t{0x7FF8000000000123}),  // quiet NaN with a payload
    std::bit_cast<double>(std::uint64_t{0x7FF0000000000001}),  // signalling NaN
    std::numeric_limits<double>::infinity(),
  };
  for (double ts : cases) {
    std::uint8_t trig[p::kTrigSize] = {};
    p::TriggerPacket tp;
    tp.entries[0] = {7, ts};
    p::encode_trigger(trig, tp);
    auto decoded = p::decode_trigger(trig, sizeof(trig));
    ASSERT_TRUE(decoded);

    std::uint8_t pose[p::kPoseSize] = {};
    p::encode_pose(pose, {1, decoded->entries[0].timestamp, 1.0, 2.0, 3.0});
    EXPECT_EQ(0, std::memcmp(pose + p::kPoseOffTimestamp, trig + p::trig_off_timestamp(0), 8))
      << std::hex << bits(ts);
  }
}

TEST(Protocol, ByteOrderIsLittleEndianRegardlessOfHost)
{
  std::uint8_t b[8];
  p::store_le_u64(b, 0x0102030405060708ULL);
  const std::uint8_t expected[8] = {8, 7, 6, 5, 4, 3, 2, 1};
  EXPECT_EQ(0, std::memcmp(b, expected, 8));
  EXPECT_EQ(p::load_le_u64(b), 0x0102030405060708ULL);
  p::store_le_i64(b, -2);
  EXPECT_EQ(b[0], 0xFE);
  EXPECT_EQ(b[7], 0xFF);
  EXPECT_EQ(p::load_le_i64(b), -2);
}

TEST(Protocol, WrongLengthIsRejected)
{
  std::uint8_t buf[128] = {};
  EXPECT_FALSE(p::decode_odometry(buf, p::kOdomSize - 1));
  EXPECT_FALSE(p::decode_odometry(buf, p::kOdomSize + 1));
  EXPECT_FALSE(p::decode_pose(buf, p::kPoseSize + 8));
  EXPECT_FALSE(p::decode_trigger(buf, p::kTrigSize - 8));
  EXPECT_TRUE(p::decode_odometry(buf, p::kOdomSize));
  EXPECT_TRUE(p::decode_pose(buf, p::kPoseSize));
  EXPECT_TRUE(p::decode_trigger(buf, p::kTrigSize));
}
