// Robot geometry and leg kinematics.
//
// Frames (all right-handed, metres, radians):
//   base_link   origin in the body centre on the femur-axis plane, x forward, z up.
//   leg frame   origin on a coxa axis, x along the leg's neutral direction.
//   stance      ground plane under the body (z = 0 on the ground), yaw-aligned
//               with base_link; the gait generator places the feet here.
//
// Joint zero: femur horizontal, tibia perpendicular to the femur (pointing
// down).  Femur and tibia rotate about -y of their frames, so positive angles
// lift the femur and swing the tibia outwards.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "hexabot_locomotion/types.hpp"

namespace hexabot_locomotion
{

struct Geometry
{
  std::array<std::string, kNumLegs> names{{"lf", "lm", "lr", "rf", "rm", "rr"}};
  std::array<LegMount, kNumLegs> mounts{};
  double coxa_length{0.0};
  double femur_length{0.0};
  double tibia_length{0.0};
  Limits coxa_limits{};
  Limits femur_limits{};
  Limits tibia_limits{};
  double stance_radius{0.0};
  double body_height{0.0};
  Limits body_height_range{};
  double step_height{0.0};
  double step_height_max{0.0};

  /// Values generated from cad/params.py.
  static Geometry defaults();

  /// Neutral foot position in the stance frame (z = 0).
  Vec3 neutralFoot(int leg) const;

  /// Joint names in controller order: <leg>_coxa_joint, <leg>_femur_joint, <leg>_tibia_joint.
  std::vector<std::string> jointNames() const;

  /// Index of a leg by name, -1 if unknown.
  int legIndex(const std::string & name) const;
};

struct IkResult
{
  JointAngles q{};
  bool reachable{false};      ///< foot inside the leg's workspace
  bool within_limits{false};  ///< all joints inside their limits (after clamping it may not be)
};

/// base_link -> leg frame
Vec3 legFrameFromBase(const Geometry & g, int leg, const Vec3 & p_base);
/// leg frame -> base_link
Vec3 baseFromLegFrame(const Geometry & g, int leg, const Vec3 & p_leg);

/// Foot position in the leg frame.
Vec3 forwardKinematicsLeg(const Geometry & g, const JointAngles & q);
/// Foot position in base_link.
Vec3 forwardKinematics(const Geometry & g, int leg, const JointAngles & q);

/// Analytic IK ("knee up" solution).  The result is clamped to the joint limits.
IkResult inverseKinematics(const Geometry & g, int leg, const Vec3 & foot_base);

/// Stance frame -> base_link for a body pose at the given height.
Vec3 stanceToBase(const Vec3 & p_stance, const BodyPose & pose, double body_height);
/// base_link -> stance frame (inverse of stanceToBase).
Vec3 baseToStance(const Vec3 & p_base, const BodyPose & pose, double body_height);

}  // namespace hexabot_locomotion
