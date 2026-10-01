#include "hexabot_locomotion/odometry.hpp"

#include <cmath>

namespace hexabot_locomotion
{

bool estimateBodyMotion(
  const std::array<Vec3, kNumLegs> & prev, const std::array<Vec3, kNumLegs> & curr,
  const std::array<bool, kNumLegs> & stance, Pose2D * delta)
{
  // World-fixed points: a_i (prev body frame) = R * b_i (curr body frame) + d.
  int n = 0;
  double ax = 0, ay = 0, bx = 0, by = 0;
  for (int i = 0; i < kNumLegs; ++i) {
    if (!stance[i]) {continue;}
    ax += prev[i].x;
    ay += prev[i].y;
    bx += curr[i].x;
    by += curr[i].y;
    ++n;
  }
  if (n < 2) {
    return false;
  }
  ax /= n;
  ay /= n;
  bx /= n;
  by /= n;
  double sxx = 0, sxy = 0;
  for (int i = 0; i < kNumLegs; ++i) {
    if (!stance[i]) {continue;}
    const double px = curr[i].x - bx, py = curr[i].y - by;
    const double qx = prev[i].x - ax, qy = prev[i].y - ay;
    sxx += px * qx + py * qy;
    sxy += px * qy - py * qx;
  }
  const double theta = std::atan2(sxy, sxx);
  const double c = std::cos(theta), s = std::sin(theta);
  delta->yaw = theta;
  delta->x = ax - (c * bx - s * by);
  delta->y = ay - (s * bx + c * by);
  return true;
}

Pose2D integratePose(const Pose2D & pose, const Pose2D & delta)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {pose.x + c * delta.x - s * delta.y, pose.y + s * delta.x + c * delta.y,
    wrapAngle(pose.yaw + delta.yaw)};
}

bool LegOdometry::update(
  const std::array<Vec3, kNumLegs> & feet, const std::array<bool, kNumLegs> & stance)
{
  bool ok = false;
  if (has_prev_) {
    // Only feet that were on the ground in both samples are world-fixed.
    std::array<bool, kNumLegs> both{};
    for (int i = 0; i < kNumLegs; ++i) {
      both[i] = stance[i] && prev_stance_[i];
    }
    Pose2D d;
    ok = estimateBodyMotion(prev_, feet, both, &d);
    if (ok) {
      last_delta_ = d;
      gap_ = 0;
    } else if (gap_++ < 3) {
      // Support swap (e.g. tripod): no foot was on the ground in both
      // samples.  Assume the body kept its velocity for this short gap.
      d = last_delta_;
      ok = true;
    }
    if (ok) {
      pose_ = integratePose(pose_, d);
    }
  }
  prev_ = feet;
  prev_stance_ = stance;
  has_prev_ = true;
  return ok;
}

void LegOdometry::reset(const Pose2D & pose)
{
  pose_ = pose;
  has_prev_ = false;
  last_delta_ = {};
  gap_ = 0;
}

}  // namespace hexabot_locomotion
