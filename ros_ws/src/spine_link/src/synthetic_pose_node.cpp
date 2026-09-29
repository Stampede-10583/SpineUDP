// SyntheticPoseNode: stands in for the adapter + TavSLAM in tests and on the bench.
//
// For each trigger on spine/triggers it makes one "frame" (frame count = triggerSeq), looks the
// frame's timestamp up in its own trigger table by exact triggerSeq (the stand-in for the
// adapter's frame-count lookup), builds a plausible field pose from Stream 1 odometry plus a
// wandering error, and publishes it as FusedPose on fused_pose after a simulated processing
// latency. Varying latency produces out-of-order results, which exercises the companion's order
// gate.
//
// Time: it never generates a protocol timestamp. Every controller_timestamp it publishes is a
// trigger's float64, copied. The Orin steady clock only schedules releases (simulated latency).
//
// Fault injection (all probabilities per trigger):
//   drop_probability     no result at all (a frame the pipeline lost)
//   invalid_probability  FusedPose with valid = false (no fix)
//   stale_probability    result held back stale_delay_s, past the companion's max_age_s

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "spine_interfaces/msg/fused_pose.hpp"
#include "spine_interfaces/msg/spine_odometry.hpp"
#include "spine_interfaces/msg/spine_trigger.hpp"
#include "spine_link/se2.hpp"

namespace spine
{

namespace msg = spine_interfaces::msg;
using SteadyTime = std::chrono::steady_clock::time_point;

class SyntheticPoseNode : public rclcpp::Node
{
public:
  explicit SyntheticPoseNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("spine_synthetic_pose", options)
  {
    field_start_ = {
      declare_parameter("field_start_x", 2.0),
      declare_parameter("field_start_y", 4.0),
      declare_parameter("field_start_theta", 0.0)};
    latency_mean_s_ = declare_parameter("latency_mean_s", 0.045);
    latency_jitter_s_ = declare_parameter("latency_jitter_s", 0.025);
    drop_p_ = declare_parameter("drop_probability", 0.0);
    invalid_p_ = declare_parameter("invalid_probability", 0.0);
    stale_p_ = declare_parameter("stale_probability", 0.0);
    stale_delay_s_ = declare_parameter("stale_delay_s", 2.0);
    ou_rate_ = declare_parameter("wander_reversion_per_s", 0.5);
    ou_sigma_xy_ = declare_parameter("wander_sigma_xy", 0.03);
    ou_sigma_theta_ = declare_parameter("wander_sigma_theta", 0.01);
    noise_xy_ = declare_parameter("noise_xy", 0.005);
    noise_theta_ = declare_parameter("noise_theta", 0.002);
    const auto seed = declare_parameter<std::int64_t>("seed", 10583);
    rng_.seed(seed != 0 ? static_cast<std::uint64_t>(seed) : std::random_device{}());

    pose_pub_ = create_publisher<msg::FusedPose>("fused_pose", rclcpp::QoS(32));
    odom_sub_ = create_subscription<msg::SpineOdometry>(
      "spine/odometry", rclcpp::QoS(200),
      [this](msg::SpineOdometry::UniquePtr m) {on_odometry(*m);});
    trig_sub_ = create_subscription<msg::SpineTrigger>(
      "spine/triggers", rclcpp::QoS(64),
      [this](msg::SpineTrigger::UniquePtr m) {on_trigger(*m);});
    release_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] {release_due();});
  }

private:
  struct OdomSample
  {
    double ts;
    Pose2 pose;
  };

  struct Pending
  {
    SteadyTime due;
    msg::FusedPose pose;
  };

  void on_odometry(const msg::SpineOdometry & m)
  {
    if (!odom_.empty() && m.controller_timestamp < odom_.back().ts) {
      reset_run();  // RIO time went backwards: new RIO run
    }
    odom_.push_back({m.controller_timestamp, {m.x, m.y, m.theta}});
    while (odom_.size() > 600) {  // 3 s at 200 Hz
      odom_.pop_front();
    }
  }

  void on_trigger(const msg::SpineTrigger & t)
  {
    if (!trigger_table_.empty() && t.trigger_seq <= trigger_table_.rbegin()->first) {
      reset_run();  // triggerSeq went backwards: new RIO run
    }
    trigger_table_[t.trigger_seq] = t.controller_timestamp;
    while (trigger_table_.size() > 128) {
      trigger_table_.erase(trigger_table_.begin());
    }

    // The camera frame this trigger exposed. Its frame count equals its triggerSeq here; a real
    // adapter derives the count from the camera and maps it with an explicit alignment.
    const std::int64_t frame_count = t.trigger_seq;

    // ===================================================================================
    // ONE TRUE CLOCK - DO NOT TOUCH THIS TIMESTAMP
    //
    // The frame's timestamp is the RIO float64 of the trigger with exactly this triggerSeq,
    // copied. No nearest match, no interpolation, no Orin clock. It travels on unchanged in
    // FusedPose.controller_timestamp and the companion writes its exact 8 bytes to Stream 2.
    // ===================================================================================
    const auto it = trigger_table_.find(frame_count);
    if (it == trigger_table_.end()) {
      return;  // no matching trigger: drop the frame, never guess
    }
    const double controller_timestamp = it->second;

    const auto sample = nearest_odometry(controller_timestamp);  // RIO time vs RIO time
    if (!sample) {
      return;  // no odometry yet
    }
    if (!odom_origin_) {
      odom_origin_ = sample->pose;
    }
    advance_wander(controller_timestamp);

    if (uniform_(rng_) < drop_p_) {
      return;
    }
    Pose2 truth = compose(field_start_, relative(*odom_origin_, sample->pose));
    msg::FusedPose out;
    out.controller_timestamp = controller_timestamp;
    out.trigger_seq = t.trigger_seq;
    out.x = truth.x + wander_.x + noise_xy_ * normal_(rng_);
    out.y = truth.y + wander_.y + noise_xy_ * normal_(rng_);
    out.theta = wrap_angle(truth.theta + wander_.theta + noise_theta_ * normal_(rng_));
    out.valid = uniform_(rng_) >= invalid_p_;

    double latency = latency_mean_s_ + latency_jitter_s_ * (2.0 * uniform_(rng_) - 1.0);
    if (uniform_(rng_) < stale_p_) {
      latency = stale_delay_s_;
    }
    const auto due = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, latency)));
    pending_.push_back({due, out});
  }

  void release_due()
  {
    const auto now = std::chrono::steady_clock::now();
    // Release in due-time order, which is not trigger order when latencies differ.
    std::stable_sort(pending_.begin(), pending_.end(),
      [](const Pending & a, const Pending & b) {return a.due < b.due;});
    while (!pending_.empty() && pending_.front().due <= now) {
      pose_pub_->publish(std::make_unique<msg::FusedPose>(pending_.front().pose));
      pending_.pop_front();
    }
  }

  std::optional<OdomSample> nearest_odometry(double ts) const
  {
    if (odom_.empty()) {
      return std::nullopt;
    }
    const auto it = std::lower_bound(odom_.begin(), odom_.end(), ts,
        [](const OdomSample & s, double t) {return s.ts < t;});
    if (it == odom_.begin()) {
      return *it;
    }
    if (it == odom_.end()) {
      return odom_.back();
    }
    const auto prev = std::prev(it);
    return (ts - prev->ts <= it->ts - ts) ? *prev : *it;
  }

  // Ornstein-Uhlenbeck random walk in x, y and theta, stepped by the RIO-time gap between
  // consecutive triggers (exact discretisation, so any dt is fine).
  void advance_wander(double trigger_ts)
  {
    if (!last_trigger_ts_) {
      last_trigger_ts_ = trigger_ts;
      return;
    }
    const double dt = std::max(0.0, trigger_ts - *last_trigger_ts_);
    last_trigger_ts_ = trigger_ts;
    // With no reversion (rate 0) this is a plain random walk: decay 1, spread sqrt(dt).
    const double decay = std::exp(-ou_rate_ * dt);
    const double spread = ou_rate_ > 0.0 ?
      std::sqrt((1.0 - decay * decay) / (2.0 * ou_rate_)) : std::sqrt(dt);
    wander_.x = wander_.x * decay + ou_sigma_xy_ * spread * normal_(rng_);
    wander_.y = wander_.y * decay + ou_sigma_xy_ * spread * normal_(rng_);
    wander_.theta = wander_.theta * decay + ou_sigma_theta_ * spread * normal_(rng_);
  }

  void reset_run()
  {
    trigger_table_.clear();
    odom_.clear();
    odom_origin_.reset();
    last_trigger_ts_.reset();
    pending_.clear();
    wander_ = {};
  }

  Pose2 field_start_;
  double latency_mean_s_, latency_jitter_s_, drop_p_, invalid_p_, stale_p_, stale_delay_s_;
  double ou_rate_, ou_sigma_xy_, ou_sigma_theta_, noise_xy_, noise_theta_;

  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::normal_distribution<double> normal_{0.0, 1.0};

  std::map<std::int64_t, double> trigger_table_;  // triggerSeq -> RIO float64
  std::deque<OdomSample> odom_;
  std::optional<Pose2> odom_origin_;
  std::optional<double> last_trigger_ts_;
  Pose2 wander_;
  std::deque<Pending> pending_;

  rclcpp::Publisher<msg::FusedPose>::SharedPtr pose_pub_;
  rclcpp::Subscription<msg::SpineOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<msg::SpineTrigger>::SharedPtr trig_sub_;
  rclcpp::TimerBase::SharedPtr release_timer_;
};

}  // namespace spine

RCLCPP_COMPONENTS_REGISTER_NODE(spine::SyntheticPoseNode)
