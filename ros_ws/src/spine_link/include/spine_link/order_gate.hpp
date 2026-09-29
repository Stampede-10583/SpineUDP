// Stream 2 ordering guarantee: never send a pose whose timestamp is <= one already sent.
//
// Why: when WPILib accepts a vision measurement at time t it deletes every vision update newer
// than t, so a late older pose rolls back newer corrections on the RIO (docs/spine-link.md
// section 4). "<=" also drops exact duplicates. Reset only on a detected RIO restart.
#pragma once

#include <cmath>

namespace spine
{

class OrderGate
{
public:
  // Returns true and records ts if it is strictly newer than every timestamp admitted so far.
  // NaN is never admitted. Admission is recorded even if the caller's send later fails: an
  // undelivered pose can't reorder anything, and a strict gate is the safe side.
  bool admit(double ts)
  {
    if (std::isnan(ts) || (has_last_ && !(ts > last_))) {
      return false;
    }
    last_ = ts;
    has_last_ = true;
    return true;
  }

  void reset() {has_last_ = false;}

  bool has_last() const {return has_last_;}
  double last() const {return last_;}

private:
  double last_ = 0.0;
  bool has_last_ = false;
};

}  // namespace spine
