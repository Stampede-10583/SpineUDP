// The companion's FusedPose -> Stream 2 path (docs/spine-link.md section 3.3), without ROS.
//
// Steps, in order; the first that fails withholds the pose (nothing is sent):
//   1. no fix       valid == false, or x/y/theta not finite
//   2. bad ts       controller_timestamp is NaN
//   3. stale        newest_trigger_ts - controller_timestamp > max_age_s, or no trigger yet
//   4. order gate   controller_timestamp <= last sent
//   5. field frame  optional static field_from_map transform of x, y, theta
//   6. encode       40 bytes; the timestamp as its exact 8 bytes
// Sending (step 7) is the caller's job.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "spine_link/order_gate.hpp"
#include "spine_link/protocol.hpp"

namespace spine
{

struct PoseInput
{
  double controller_timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  bool valid = false;
};

// Static 2-D transform from TavSLAM's map frame to the WPILib field frame.
struct FieldFromMap
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  bool is_identity() const {return x == 0.0 && y == 0.0 && yaw == 0.0;}
};

class PosePipeline
{
public:
  enum class Verdict { kSend, kNoFix, kBadTimestamp, kStale, kOutOfOrder };

  PosePipeline(double max_age_s, FieldFromMap field_from_map)
  : max_age_s_(max_age_s), field_from_map_(field_from_map) {}

  // Runs steps 1-6. On kSend, `out` holds the Stream 2 datagram and the order gate has advanced.
  // `has_trigger` / `newest_trigger_ts` come from the TriggerTable (RIO time only).
  Verdict process(
    const PoseInput & in, bool has_trigger, double newest_trigger_ts,
    std::array<std::uint8_t, protocol::kPoseSize> & out)
  {
    if (!in.valid || !std::isfinite(in.x) || !std::isfinite(in.y) || !std::isfinite(in.theta)) {
      return Verdict::kNoFix;
    }
    if (std::isnan(in.controller_timestamp)) {
      return Verdict::kBadTimestamp;
    }
    // RIO time compared only with RIO time. !(a <= b) also catches +/-inf.
    if (!has_trigger || !(newest_trigger_ts - in.controller_timestamp <= max_age_s_)) {
      return Verdict::kStale;
    }
    if (!gate_.admit(in.controller_timestamp)) {
      return Verdict::kOutOfOrder;
    }

    double x = in.x;
    double y = in.y;
    double theta = in.theta;
    if (!field_from_map_.is_identity()) {
      const double c = std::cos(field_from_map_.yaw);
      const double s = std::sin(field_from_map_.yaw);
      x = field_from_map_.x + c * in.x - s * in.y;
      y = field_from_map_.y + s * in.x + c * in.y;
      const double t = in.theta + field_from_map_.yaw;
      theta = std::atan2(std::sin(t), std::cos(t));
    }

    std::uint8_t * d = out.data();
    protocol::store_le_i64(d + protocol::kPoseOffSeq, next_seq_++);

    // ===================================================================================
    // ONE TRUE CLOCK - DO NOT TOUCH THIS TIMESTAMP
    //
    // controller_timestamp is the RIO's own float64 from Stream 3, carried unchanged through
    // the adapter and TavSLAM. It is written here as its exact 8 bytes: a bit copy, no
    // arithmetic. Never recompute it, add latency to it, round it, convert it through
    // builtin_interfaces/Time, or replace it with any Orin clock. A wrong value raises no
    // error on the RIO; it silently shifts every correction in time.
    // ===================================================================================
    protocol::store_le_f64(d + protocol::kPoseOffTimestamp, in.controller_timestamp);

    protocol::store_le_f64(d + protocol::kPoseOffX, x);
    protocol::store_le_f64(d + protocol::kPoseOffY, y);
    protocol::store_le_f64(d + protocol::kPoseOffTheta, theta);
    return Verdict::kSend;
  }

  // Called when a RIO restart is detected, with the new run's newest trigger timestamp.
  // A robot-program restart keeps the RIO clock running, so the gate is kept and nothing older
  // than an already-sent pose can go out. Only a reboot, where RIO time went backwards, reopens
  // the gate; otherwise the new run could never send anything.
  void on_rio_restart(double new_run_newest_ts)
  {
    if (gate_.has_last() && !(new_run_newest_ts > gate_.last())) {
      gate_.reset();
    }
  }

  const OrderGate & gate() const {return gate_;}

private:
  double max_age_s_;
  FieldFromMap field_from_map_;
  OrderGate gate_;
  std::int64_t next_seq_ = 0;
};

}  // namespace spine
