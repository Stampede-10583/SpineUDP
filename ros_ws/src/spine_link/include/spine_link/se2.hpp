// Minimal SE(2) pose algebra for the synthetic pose generator.
#pragma once

#include <cmath>

namespace spine
{

struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

inline double wrap_angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

// a (+) b: b expressed in a's frame, mapped out to a's parent frame.
inline Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_angle(a.theta + b.theta)};
}

// from^-1 (+) to: the pose of `to` in `from`'s frame.
inline Pose2 relative(const Pose2 & from, const Pose2 & to)
{
  const double c = std::cos(from.theta);
  const double s = std::sin(from.theta);
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return {c * dx + s * dy, -s * dx + c * dy, wrap_angle(to.theta - from.theta)};
}

}  // namespace spine
