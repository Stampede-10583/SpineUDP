// Stream 3 read rule and RIO-restart detection (docs/spine-link.md sections 2 and 3).
// No ROS dependency, no clock: every decision uses RIO data only.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "spine_link/protocol.hpp"

namespace spine
{

class TriggerTable
{
public:
  enum class Outcome
  {
    kNew,          // one or more new triggers (see Result::fresh)
    kNothingNew,   // duplicate, reordered, or no triggers recorded yet
    kDecodeError,  // entries not consecutive/descending; nothing applied
  };

  struct Result
  {
    Outcome outcome = Outcome::kNothingNew;
    bool restarted = false;       // a RIO restart was detected before walking this packet
    std::int64_t missed = 0;      // triggerSeq values skipped (5+ packets lost in a row)
    std::size_t fresh_count = 0;  // number of valid entries in fresh
    std::array<protocol::TriggerEntry, protocol::kTrigEntryCount> fresh{};  // oldest first
  };

  // Applies one decoded Stream 3 packet.
  Result process(const protocol::TriggerPacket & packet);

  // Forgets everything, as after a RIO restart.
  void reset();

  // True once any trigger has been seen since construction or the last restart/reset.
  bool has_trigger() const {return last_seen_ != protocol::kEmptyTriggerSeq;}

  // triggerSeq and RIO timestamp of the newest trigger seen. Valid only if has_trigger().
  std::int64_t last_seen_seq() const {return last_seen_;}
  double newest_timestamp() const {return last_seen_ts_;}

  // Checks the RIO's within-packet guarantee: real entries first, consecutive and descending by
  // exactly 1, with non-decreasing timestamps toward entry 0; then only (-1, 0.0) sentinels.
  static bool is_consistent(const protocol::TriggerPacket & packet);

private:
  std::int64_t last_seen_ = protocol::kEmptyTriggerSeq;
  double last_seen_ts_ = 0.0;
};

}  // namespace spine
