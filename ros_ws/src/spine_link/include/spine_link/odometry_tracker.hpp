// Stream 1 bookkeeping for the companion: sequence gaps, duplicates/reordering, RIO restarts.
// RIO data only; no clock.
#pragma once

#include <cstdint>

namespace spine
{

class OdometryTracker
{
public:
  enum class Outcome
  {
    kAccept,   // publish it
    kRestart,  // new RIO run; tracking was reset and this packet starts it (publish it)
    kStale,    // duplicate or reordered older packet; drop it
  };

  struct Result
  {
    Outcome outcome = Outcome::kAccept;
    std::int64_t gap = 0;  // seq values skipped before this packet
  };

  // Packets further back than this can't be LAN reordering (0.25 s at 200 Hz).
  static constexpr std::int64_t kReorderWindow = 50;

  Result process(std::int64_t seq, double timestamp)
  {
    Result r;
    if (has_last_) {
      // Within one RIO run, seq and RIO time rise together; any disagreement is a new run.
      const bool restarted =
        seq < last_seq_ - kReorderWindow ||
        (seq <= last_seq_ && timestamp > last_ts_) ||
        (seq > last_seq_ && timestamp < last_ts_);
      if (restarted) {
        r.outcome = Outcome::kRestart;
      } else if (seq <= last_seq_) {
        r.outcome = Outcome::kStale;
        return r;
      } else if (seq > last_seq_ + 1) {
        r.gap = seq - last_seq_ - 1;
      }
    }
    has_last_ = true;
    last_seq_ = seq;
    last_ts_ = timestamp;
    return r;
  }

private:
  bool has_last_ = false;
  std::int64_t last_seq_ = 0;
  double last_ts_ = 0.0;
};

}  // namespace spine
