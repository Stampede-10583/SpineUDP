// SpineCompanionNode: the Orin end of the Spine link (docs/spine-link.md).
//
//   Stream 1 (UDP :5801) -> spine/odometry  (SpineOdometry)
//   Stream 3 (UDP :5803) -> spine/triggers  (SpineTrigger, each new trigger once, oldest first)
//   fused_pose (FusedPose) -> Stream 2 (UDP rio_address:5802)
//   spine/stats (LinkStats) and /diagnostics every stats_period_s; spine/events (LinkEvent)
//
// Threads: one receive thread epolls the Stream 1 and Stream 3 sockets; the fused_pose callback
// runs on the executor. They share the trigger table and pose pipeline under one mutex.
//
// Time: every protocol decision uses RIO time only. The Orin clock is used for nothing but the
// stats cadence and diagnostic stamps.

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "spine_interfaces/msg/fused_pose.hpp"
#include "spine_interfaces/msg/link_event.hpp"
#include "spine_interfaces/msg/link_stats.hpp"
#include "spine_interfaces/msg/spine_odometry.hpp"
#include "spine_interfaces/msg/spine_trigger.hpp"
#include "spine_link/odometry_tracker.hpp"
#include "spine_link/pose_pipeline.hpp"
#include "spine_link/protocol.hpp"
#include "spine_link/trigger_table.hpp"

namespace spine
{

namespace msg = spine_interfaces::msg;

namespace
{

int bind_udp(const std::string & address, int port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  const int rcvbuf = 1 << 18;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (::inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    ::close(fd);
    throw std::runtime_error("bind_address must be an IPv4 address: " + address);
  }
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("bind " + address + ":" + std::to_string(port) + ": " + err);
  }
  return fd;
}

sockaddr_in resolve_ipv4(const std::string & host, int port)
{
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo * res = nullptr;
  if (::getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || res == nullptr) {
    throw std::runtime_error("cannot resolve rio_address " + host);
  }
  sockaddr_in addr = *reinterpret_cast<sockaddr_in *>(res->ai_addr);
  ::freeaddrinfo(res);
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  return addr;
}

}  // namespace

class SpineCompanionNode : public rclcpp::Node
{
public:
  explicit SpineCompanionNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("spine_companion", options),
    pipeline_(
      declare_parameter("max_age_s", 1.4),
      FieldFromMap{
        declare_parameter("field_from_map_x", 0.0),
        declare_parameter("field_from_map_y", 0.0),
        declare_parameter("field_from_map_yaw", 0.0)})
  {
    const auto bind_address = declare_parameter("bind_address", std::string("0.0.0.0"));
    const auto odom_port = declare_parameter<std::int64_t>("odometry_port", protocol::kOdometryPort);
    const auto trig_port = declare_parameter<std::int64_t>("trigger_port", protocol::kTriggerTablePort);
    const auto rio_address = declare_parameter("rio_address", std::string("10.105.83.2"));
    const auto rio_port = declare_parameter<std::int64_t>("rio_pose_port", protocol::kVisionPosePort);
    const auto stats_period = declare_parameter("stats_period_s", 1.0);

    odom_pub_ = create_publisher<msg::SpineOdometry>("spine/odometry", rclcpp::QoS(200));
    trig_pub_ = create_publisher<msg::SpineTrigger>("spine/triggers", rclcpp::QoS(64));
    event_pub_ = create_publisher<msg::LinkEvent>(
      "spine/events", rclcpp::QoS(100).reliable().transient_local());
    stats_pub_ = create_publisher<msg::LinkStats>("spine/stats", rclcpp::QoS(10));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    pose_sub_ = create_subscription<msg::FusedPose>(
      "fused_pose", rclcpp::QoS(32),
      [this](msg::FusedPose::UniquePtr m) {on_fused_pose(*m);});

    rio_addr_ = resolve_ipv4(rio_address, static_cast<int>(rio_port));
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    odom_fd_ = bind_udp(bind_address, static_cast<int>(odom_port));
    trig_fd_ = bind_udp(bind_address, static_cast<int>(trig_port));
    wake_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (send_fd_ < 0 || wake_fd_ < 0) {
      throw std::runtime_error(std::string("socket/eventfd: ") + std::strerror(errno));
    }

    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period), [this] {publish_stats();});

    rx_thread_ = std::thread([this] {receive_loop();});
    RCLCPP_INFO(
      get_logger(), "listening on %s:%ld (odometry) and :%ld (triggers); poses -> %s:%ld",
      bind_address.c_str(), static_cast<long>(odom_port), static_cast<long>(trig_port),
      rio_address.c_str(), static_cast<long>(rio_port));
  }

  ~SpineCompanionNode() override
  {
    running_ = false;
    const std::uint64_t one = 1;
    [[maybe_unused]] auto n = ::write(wake_fd_, &one, sizeof(one));
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    for (int fd : {odom_fd_, trig_fd_, send_fd_, wake_fd_}) {
      if (fd >= 0) {
        ::close(fd);
      }
    }
  }

private:
  // ---------------------------------------------------------------------------------------
  // Stream 2: fused_pose -> RIO
  // ---------------------------------------------------------------------------------------
  void on_fused_pose(const msg::FusedPose & m)
  {
    const PoseInput in{m.controller_timestamp, m.x, m.y, m.theta, m.valid};
    std::array<std::uint8_t, protocol::kPoseSize> out{};
    PosePipeline::Verdict v;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v = pipeline_.process(in, table_.has_trigger(), table_.newest_timestamp(), out);
    }
    switch (v) {
      case PosePipeline::Verdict::kNoFix: ++withheld_no_fix_; return;
      case PosePipeline::Verdict::kBadTimestamp: ++withheld_bad_timestamp_; return;
      case PosePipeline::Verdict::kStale: ++withheld_stale_; return;
      case PosePipeline::Verdict::kOutOfOrder: ++withheld_out_of_order_; return;
      case PosePipeline::Verdict::kSend: break;
    }
    const auto n = ::sendto(
      send_fd_, out.data(), out.size(), 0, reinterpret_cast<const sockaddr *>(&rio_addr_),
      sizeof(rio_addr_));
    if (n == static_cast<ssize_t>(out.size())) {
      ++poses_sent_;
    } else {
      ++send_errors_;
    }
  }

  // ---------------------------------------------------------------------------------------
  // Streams 1 and 3: RIO -> companion
  // ---------------------------------------------------------------------------------------
  void receive_loop()
  {
    const int ep = ::epoll_create1(EPOLL_CLOEXEC);
    for (int fd : {odom_fd_, trig_fd_, wake_fd_}) {
      epoll_event ev{};
      ev.events = EPOLLIN;
      ev.data.fd = fd;
      ::epoll_ctl(ep, EPOLL_CTL_ADD, fd, &ev);
    }
    std::array<epoll_event, 3> events{};
    // One byte larger than any valid packet, and MSG_TRUNC reports the real datagram length,
    // so an oversized datagram is counted as a decode error instead of being truncated.
    std::array<std::uint8_t, protocol::kTrigSize + 1> buf{};
    while (running_) {
      const int n = ::epoll_wait(ep, events.data(), static_cast<int>(events.size()), 500);
      for (int i = 0; i < n; ++i) {
        const int fd = events[static_cast<std::size_t>(i)].data.fd;
        if (fd == wake_fd_) {
          continue;
        }
        while (true) {
          const ssize_t len = ::recv(fd, buf.data(), buf.size(), MSG_TRUNC);
          if (len < 0) {
            break;  // EAGAIN: drained
          }
          if (fd == odom_fd_) {
            on_odometry_datagram(buf.data(), static_cast<std::size_t>(len));
          } else {
            on_trigger_datagram(buf.data(), static_cast<std::size_t>(len));
          }
        }
      }
    }
    ::close(ep);
  }

  void on_odometry_datagram(const std::uint8_t * d, std::size_t len)
  {
    const auto p = protocol::decode_odometry(d, len);
    if (!p) {
      ++decode_errors_;
      emit_event(msg::LinkEvent::DECODE_ERROR, kNaN,
        "Stream 1 datagram of " + std::to_string(len) + " B");
      return;
    }
    ++odom_recvd_;
    const auto r = odom_tracker_.process(p->seq, p->timestamp);
    if (r.outcome == OdometryTracker::Outcome::kStale) {
      return;
    }
    if (r.outcome == OdometryTracker::Outcome::kRestart) {
      // Counted in rio_restarts only when Stream 3 sees it, since that is what resets state.
      emit_event(msg::LinkEvent::RIO_RESTART, p->timestamp, "detected on Stream 1");
    }
    if (r.gap > 0) {
      odom_seq_gaps_ += static_cast<std::uint64_t>(r.gap);
      emit_event(msg::LinkEvent::ODOM_GAP, p->timestamp,
        std::to_string(r.gap) + " Stream 1 packets missing");
    }
    auto m = std::make_unique<msg::SpineOdometry>();
    m->seq = p->seq;
    m->controller_timestamp = p->timestamp;
    m->x = p->x;
    m->y = p->y;
    m->theta = p->theta;
    m->vx = p->vx;
    m->vy = p->vy;
    m->omega = p->omega;
    odom_pub_->publish(std::move(m));
  }

  void on_trigger_datagram(const std::uint8_t * d, std::size_t len)
  {
    const auto p = protocol::decode_trigger(d, len);
    if (!p) {
      ++decode_errors_;
      emit_event(msg::LinkEvent::DECODE_ERROR, kNaN,
        "Stream 3 datagram of " + std::to_string(len) + " B");
      return;
    }
    ++trig_recvd_;
    TriggerTable::Result r;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      r = table_.process(*p);
      if (r.restarted) {
        pipeline_.on_rio_restart(table_.newest_timestamp());
      }
    }
    if (r.outcome == TriggerTable::Outcome::kDecodeError) {
      ++decode_errors_;
      emit_event(msg::LinkEvent::DECODE_ERROR, p->entries[0].timestamp,
        "inconsistent Stream 3 table, packetSeq " + std::to_string(p->packet_seq));
      return;
    }
    if (r.restarted) {
      ++rio_restarts_;
      last_restart_ts_ = r.fresh[0].timestamp;
      emit_event(msg::LinkEvent::RIO_RESTART, r.fresh[0].timestamp,
        "detected on Stream 3; trigger history reset");
    }
    if (r.missed > 0) {
      triggers_missed_ += static_cast<std::uint64_t>(r.missed);
      emit_event(msg::LinkEvent::TRIGGER_GAP, r.fresh[0].timestamp,
        std::to_string(r.missed) + " triggers never received");
    }
    for (std::size_t i = 0; i < r.fresh_count; ++i) {
      auto m = std::make_unique<msg::SpineTrigger>();
      m->trigger_seq = r.fresh[i].trigger_seq;
      m->packet_seq = p->packet_seq;
      m->controller_timestamp = r.fresh[i].timestamp;
      trig_pub_->publish(std::move(m));
    }
  }

  // ---------------------------------------------------------------------------------------
  // Stats, diagnostics and events
  // ---------------------------------------------------------------------------------------
  void emit_event(std::uint8_t kind, double controller_ts, const std::string & detail)
  {
    auto m = std::make_unique<msg::LinkEvent>();
    m->stamp = now();
    m->kind = kind;
    m->controller_timestamp = controller_ts;
    m->detail = detail;
    event_pub_->publish(std::move(m));
  }

  void publish_stats()
  {
    auto s = std::make_unique<msg::LinkStats>();
    s->stamp = now();
    s->poses_sent = poses_sent_;
    s->withheld_no_fix = withheld_no_fix_;
    s->withheld_bad_timestamp = withheld_bad_timestamp_;
    s->withheld_stale = withheld_stale_;
    s->withheld_out_of_order = withheld_out_of_order_;
    s->send_errors = send_errors_;
    s->odom_recvd = odom_recvd_;
    s->odom_seq_gaps = odom_seq_gaps_;
    s->trig_recvd = trig_recvd_;
    s->triggers_missed = triggers_missed_;
    s->decode_errors = decode_errors_;
    s->rio_restarts = rio_restarts_;
    s->last_restart_controller_timestamp = last_restart_ts_;

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = s->stamp;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "spine_link: companion";
    st.hardware_id = "spine";
    const bool odom_flowing = s->odom_recvd != prev_odom_recvd_;
    const bool trig_flowing = s->trig_recvd != prev_trig_recvd_;
    prev_odom_recvd_ = s->odom_recvd;
    prev_trig_recvd_ = s->trig_recvd;
    if (odom_flowing && trig_flowing) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = "RIO streams flowing";
    } else {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = !odom_flowing && !trig_flowing ? "no traffic from the RIO" :
        (!odom_flowing ? "no Stream 1 traffic" : "no Stream 3 traffic");
    }
    const auto kv = [&st](const char * k, std::uint64_t v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = std::to_string(v);
        st.values.push_back(std::move(e));
      };
    kv("posesSent", s->poses_sent);
    kv("withheldNoFix", s->withheld_no_fix);
    kv("withheldBadTimestamp", s->withheld_bad_timestamp);
    kv("withheldStale", s->withheld_stale);
    kv("withheldOutOfOrder", s->withheld_out_of_order);
    kv("sendErrors", s->send_errors);
    kv("odomRecvd", s->odom_recvd);
    kv("odomSeqGaps", s->odom_seq_gaps);
    kv("trigRecvd", s->trig_recvd);
    kv("triggersMissed", s->triggers_missed);
    kv("decodeErrors", s->decode_errors);
    kv("rioRestarts", s->rio_restarts);
    diag.status.push_back(std::move(st));

    stats_pub_->publish(std::move(s));
    diag_pub_->publish(diag);
  }

  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // Shared between the receive thread and the fused_pose callback.
  std::mutex mutex_;
  TriggerTable table_;
  PosePipeline pipeline_;

  // Receive thread only.
  OdometryTracker odom_tracker_;

  std::atomic<std::uint64_t> poses_sent_{0}, withheld_no_fix_{0}, withheld_bad_timestamp_{0},
    withheld_stale_{0}, withheld_out_of_order_{0}, send_errors_{0}, odom_recvd_{0},
    odom_seq_gaps_{0}, trig_recvd_{0}, triggers_missed_{0}, decode_errors_{0}, rio_restarts_{0};
  std::atomic<double> last_restart_ts_{kNaN};
  std::uint64_t prev_odom_recvd_ = 0;
  std::uint64_t prev_trig_recvd_ = 0;

  sockaddr_in rio_addr_{};
  int odom_fd_ = -1;
  int trig_fd_ = -1;
  int send_fd_ = -1;
  int wake_fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread rx_thread_;

  rclcpp::Publisher<msg::SpineOdometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<msg::SpineTrigger>::SharedPtr trig_pub_;
  rclcpp::Publisher<msg::LinkEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<msg::LinkStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<msg::FusedPose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace spine

RCLCPP_COMPONENTS_REGISTER_NODE(spine::SpineCompanionNode)
