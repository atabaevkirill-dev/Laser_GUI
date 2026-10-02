// Leg odometry: body motion from the feet that stay on the ground.
#pragma once

#include <array>

#include "hexabot_locomotion/types.hpp"

namespace hexabot_locomotion
{

/// Planar rigid motion of the body between two samples of foot positions
/// (both in the body frame).  Feet in stance are fixed in the world, so the
/// body moved by (dx, dy, dyaw) expressed in the body frame of the first
/// sample.  Needs at least two stance feet; returns false otherwise.
bool estimateBodyMotion(
  const std::array<Vec3, kNumLegs> & prev, const std::array<Vec3, kNumLegs> & curr,
  const std::array<bool, kNumLegs> & stance, Pose2D * delta);

/// Composes a body-frame increment onto a world pose.
Pose2D integratePose(const Pose2D & pose, const Pose2D & delta);

class LegOdometry
{
public:
  /// Feed foot positions (body frame) and stance flags; returns false while
  /// fewer than two feet are on the ground.
  bool update(const std::array<Vec3, kNumLegs> & feet, const std::array<bool, kNumLegs> & stance);
  void reset(const Pose2D & pose = {});
  const Pose2D & pose() const {return pose_;}
  const Pose2D & lastDelta() const {return last_delta_;}

private:
  bool has_prev_{false};
  std::array<Vec3, kNumLegs> prev_{};
  std::array<bool, kNumLegs> prev_stance_{};
  Pose2D pose_{};
  Pose2D last_delta_{};
  int gap_{0};
};

}  // namespace hexabot_locomotion
