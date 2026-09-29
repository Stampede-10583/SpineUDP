// Spine link wire format: the coprocessor-side mirror of VisionLinkProtocol in
// src/main/java/frc/lib/SpineUDP/SpineUDP.java. Header-only, no ROS dependency.
//
// Keep in lockstep with VisionLinkProtocol, docs/spine-link.md and
// test/data/spine-link-vectors.txt. There is no version field on the wire.
//
// Rules this file holds to:
//   - Little-endian on the wire, always, independent of host byte order. Every field is loaded
//     and stored byte by byte at its absolute offset.
//   - Never cast a struct over a buffer.
//   - float64 fields move as their raw IEEE-754 bits (std::bit_cast to uint64_t), so a controller
//     timestamp read from Stream 3 and written to Stream 2 is bit-for-bit identical, NaN payloads
//     and -0.0 included. No arithmetic ever touches a timestamp here.
#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace spine::protocol
{

static_assert(std::numeric_limits<double>::is_iec559, "float64 fields must be IEEE-754 doubles");
static_assert(sizeof(double) == 8 && sizeof(std::uint64_t) == 8);

// ------------------------------------------------------------------------------------------
// Stream 1 - odometry (RIO -> Orin), 64 bytes
// ------------------------------------------------------------------------------------------
inline constexpr std::size_t kOdomSize = 64;
inline constexpr std::size_t kOdomOffSeq = 0;
inline constexpr std::size_t kOdomOffTimestamp = 8;
inline constexpr std::size_t kOdomOffX = 16;
inline constexpr std::size_t kOdomOffY = 24;
inline constexpr std::size_t kOdomOffTheta = 32;
inline constexpr std::size_t kOdomOffVx = 40;
inline constexpr std::size_t kOdomOffVy = 48;
inline constexpr std::size_t kOdomOffOmega = 56;

// ------------------------------------------------------------------------------------------
// Stream 2 - vision pose (Orin -> RIO), 40 bytes
// ------------------------------------------------------------------------------------------
inline constexpr std::size_t kPoseSize = 40;
inline constexpr std::size_t kPoseOffSeq = 0;
inline constexpr std::size_t kPoseOffTimestamp = 8;
inline constexpr std::size_t kPoseOffX = 16;
inline constexpr std::size_t kPoseOffY = 24;
inline constexpr std::size_t kPoseOffTheta = 32;

// ------------------------------------------------------------------------------------------
// Stream 3 - trigger table (RIO -> Orin), 88 bytes
// ------------------------------------------------------------------------------------------
inline constexpr std::size_t kTrigEntryCount = 5;
inline constexpr std::size_t kTrigEntryBase = 8;
inline constexpr std::size_t kTrigEntryStride = 16;
inline constexpr std::size_t kTrigSize = kTrigEntryBase + kTrigEntryCount * kTrigEntryStride;
inline constexpr std::size_t kTrigOffPacketSeq = 0;
inline constexpr std::int64_t kEmptyTriggerSeq = -1;  // unfilled slot; its timestamp is 0.0

constexpr std::size_t trig_off_trigger_seq(std::size_t i)
{
  return kTrigEntryBase + i * kTrigEntryStride;
}
constexpr std::size_t trig_off_timestamp(std::size_t i)
{
  return kTrigEntryBase + i * kTrigEntryStride + 8;
}

// Default ports (docs/spine-link.md section 1).
inline constexpr std::uint16_t kOdometryPort = 5801;
inline constexpr std::uint16_t kVisionPosePort = 5802;
inline constexpr std::uint16_t kTriggerTablePort = 5803;

static_assert(kOdomOffOmega + 8 == kOdomSize);
static_assert(kPoseOffTheta + 8 == kPoseSize);
static_assert(kTrigSize == 88);
static_assert(trig_off_timestamp(kTrigEntryCount - 1) + 8 == kTrigSize);
static_assert(kOdomSize % 8 == 0 && kPoseSize % 8 == 0 && kTrigSize % 8 == 0);

// ------------------------------------------------------------------------------------------
// Little-endian field access, independent of host byte order
// ------------------------------------------------------------------------------------------
inline std::uint64_t load_le_u64(const std::uint8_t * p)
{
  std::uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8) | p[i];
  }
  return v;
}

inline void store_le_u64(std::uint8_t * p, std::uint64_t v)
{
  for (int i = 0; i < 8; ++i) {
    p[i] = static_cast<std::uint8_t>(v >> (8 * i));
  }
}

inline std::int64_t load_le_i64(const std::uint8_t * p)
{
  return std::bit_cast<std::int64_t>(load_le_u64(p));
}

inline void store_le_i64(std::uint8_t * p, std::int64_t v)
{
  store_le_u64(p, std::bit_cast<std::uint64_t>(v));
}

inline double load_le_f64(const std::uint8_t * p)
{
  return std::bit_cast<double>(load_le_u64(p));
}

inline void store_le_f64(std::uint8_t * p, double v)
{
  store_le_u64(p, std::bit_cast<std::uint64_t>(v));
}

// ------------------------------------------------------------------------------------------
// Decoded packets
// ------------------------------------------------------------------------------------------
struct OdometryPacket
{
  std::int64_t seq = 0;
  double timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
};

struct PosePacket
{
  std::int64_t seq = 0;
  double timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct TriggerEntry
{
  std::int64_t trigger_seq = kEmptyTriggerSeq;
  double timestamp = 0.0;
};

struct TriggerPacket
{
  std::int64_t packet_seq = 0;
  std::array<TriggerEntry, kTrigEntryCount> entries{};  // entries[0] is the most recent
};

// Decoders return nullopt unless the datagram is exactly the stream's size.
inline std::optional<OdometryPacket> decode_odometry(const std::uint8_t * d, std::size_t len)
{
  if (len != kOdomSize) {
    return std::nullopt;
  }
  OdometryPacket p;
  p.seq = load_le_i64(d + kOdomOffSeq);
  p.timestamp = load_le_f64(d + kOdomOffTimestamp);
  p.x = load_le_f64(d + kOdomOffX);
  p.y = load_le_f64(d + kOdomOffY);
  p.theta = load_le_f64(d + kOdomOffTheta);
  p.vx = load_le_f64(d + kOdomOffVx);
  p.vy = load_le_f64(d + kOdomOffVy);
  p.omega = load_le_f64(d + kOdomOffOmega);
  return p;
}

inline std::optional<PosePacket> decode_pose(const std::uint8_t * d, std::size_t len)
{
  if (len != kPoseSize) {
    return std::nullopt;
  }
  PosePacket p;
  p.seq = load_le_i64(d + kPoseOffSeq);
  p.timestamp = load_le_f64(d + kPoseOffTimestamp);
  p.x = load_le_f64(d + kPoseOffX);
  p.y = load_le_f64(d + kPoseOffY);
  p.theta = load_le_f64(d + kPoseOffTheta);
  return p;
}

inline std::optional<TriggerPacket> decode_trigger(const std::uint8_t * d, std::size_t len)
{
  if (len != kTrigSize) {
    return std::nullopt;
  }
  TriggerPacket p;
  p.packet_seq = load_le_i64(d + kTrigOffPacketSeq);
  for (std::size_t i = 0; i < kTrigEntryCount; ++i) {
    p.entries[i].trigger_seq = load_le_i64(d + trig_off_trigger_seq(i));
    p.entries[i].timestamp = load_le_f64(d + trig_off_timestamp(i));
  }
  return p;
}

inline void encode_odometry(std::uint8_t * d, const OdometryPacket & p)
{
  store_le_i64(d + kOdomOffSeq, p.seq);
  store_le_f64(d + kOdomOffTimestamp, p.timestamp);
  store_le_f64(d + kOdomOffX, p.x);
  store_le_f64(d + kOdomOffY, p.y);
  store_le_f64(d + kOdomOffTheta, p.theta);
  store_le_f64(d + kOdomOffVx, p.vx);
  store_le_f64(d + kOdomOffVy, p.vy);
  store_le_f64(d + kOdomOffOmega, p.omega);
}

inline void encode_pose(std::uint8_t * d, const PosePacket & p)
{
  store_le_i64(d + kPoseOffSeq, p.seq);
  store_le_f64(d + kPoseOffTimestamp, p.timestamp);
  store_le_f64(d + kPoseOffX, p.x);
  store_le_f64(d + kPoseOffY, p.y);
  store_le_f64(d + kPoseOffTheta, p.theta);
}

inline void encode_trigger(std::uint8_t * d, const TriggerPacket & p)
{
  store_le_i64(d + kTrigOffPacketSeq, p.packet_seq);
  for (std::size_t i = 0; i < kTrigEntryCount; ++i) {
    store_le_i64(d + trig_off_trigger_seq(i), p.entries[i].trigger_seq);
    store_le_f64(d + trig_off_timestamp(i), p.entries[i].timestamp);
  }
}

}  // namespace spine::protocol
