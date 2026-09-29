#include "spine_link/trigger_table.hpp"

#include <cmath>

namespace spine
{

using protocol::kEmptyTriggerSeq;
using protocol::kTrigEntryCount;

bool TriggerTable::is_consistent(const protocol::TriggerPacket & packet)
{
  bool in_sentinels = false;
  for (std::size_t i = 0; i < kTrigEntryCount; ++i) {
    const auto & e = packet.entries[i];
    if (e.trigger_seq == kEmptyTriggerSeq) {
      if (e.timestamp != 0.0) {
        return false;
      }
      in_sentinels = true;
      continue;
    }
    if (in_sentinels || e.trigger_seq < 0 || !std::isfinite(e.timestamp)) {
      return false;
    }
    if (i > 0) {
      const auto & newer = packet.entries[i - 1];
      if (e.trigger_seq != newer.trigger_seq - 1 || e.timestamp > newer.timestamp) {
        return false;
      }
    }
  }
  return true;
}

void TriggerTable::reset()
{
  last_seen_ = kEmptyTriggerSeq;
  last_seen_ts_ = 0.0;
}

TriggerTable::Result TriggerTable::process(const protocol::TriggerPacket & packet)
{
  Result r;
  if (!is_consistent(packet)) {
    r.outcome = Outcome::kDecodeError;
    return r;
  }
  const auto & newest = packet.entries[0];
  if (newest.trigger_seq == kEmptyTriggerSeq) {
    return r;  // the RIO has recorded no triggers yet
  }

  // Restart detection, before the walk, on RIO data only. Within one RIO run, triggerSeq and RIO
  // time rise together, so any disagreement between them means a new run:
  //   - seq went back by more than a table's worth: LAN reordering can't do that;
  //   - seq went back (or repeated) but RIO time went forward: a reordered packet can't do that;
  //   - seq went forward but RIO time went back: the RIO rebooted and its clock restarted.
  if (has_trigger()) {
    const std::int64_t s0 = newest.trigger_seq;
    const double t0 = newest.timestamp;
    const bool restarted =
      s0 < last_seen_ - static_cast<std::int64_t>(kTrigEntryCount) ||
      (s0 <= last_seen_ && t0 > last_seen_ts_) ||
      (s0 > last_seen_ && t0 < last_seen_ts_);
    if (restarted) {
      reset();
      r.restarted = true;
    } else if (s0 <= last_seen_) {
      return r;  // duplicate or reordered older packet
    }
  }

  // Walk oldest to newest. Entries rise as i falls, so updating last_seen_ inside the loop is
  // correct, and the -1 sentinel fails "> last_seen_" automatically because last_seen_ >= -1.
  for (std::size_t k = kTrigEntryCount; k-- > 0; ) {
    const auto & e = packet.entries[k];
    if (e.trigger_seq <= last_seen_) {
      continue;
    }
    if (r.fresh_count == 0 && last_seen_ != kEmptyTriggerSeq && e.trigger_seq > last_seen_ + 1) {
      r.missed = e.trigger_seq - last_seen_ - 1;
    }
    r.fresh[r.fresh_count++] = e;
    last_seen_ = e.trigger_seq;
    last_seen_ts_ = e.timestamp;
  }
  r.outcome = r.fresh_count > 0 ? Outcome::kNew : Outcome::kNothingNew;
  return r;
}

}  // namespace spine
