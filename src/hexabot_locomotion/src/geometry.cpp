#include "hexabot_locomotion/geometry.hpp"

#include <cmath>

#include "hexabot_locomotion/geometry_defaults.hpp"

namespace hexabot_locomotion
{

Geometry Geometry::defaults()
{
  Geometry g;
  for (int i = 0; i < kNumLegs; ++i) {
    g.mounts[i] = {defaults::kMountX[i], defaults::kMountY[i], defaults::kMountYaw[i]};
  }
  g.coxa_length = defaults::kCoxaLength;
  g.femur_length = defaults::kFemurLength;
  g.tibia_length = defaults::kTibiaLength;
  g.coxa_limits = {defaults::kCoxaLimits[0], defaults::kCoxaLimits[1]};
  g.femur_limits = {defaults::kFemurLimits[0], defaults::kFemurLimits[1]};
  g.tibia_limits = {defaults::kTibiaLimits[0], defaults::kTibiaLimits[1]};
  g.stance_radius = defaults::kStanceRadius;
  g.body_height = defaults::kBodyHeight;
  g.body_height_range = {defaults::kBodyHeightRange[0], defaults::kBodyHeightRange[1]};
  g.step_height = defaults::kStepHeight;
  g.step_height_max = defaults::kStepHeightMax;
  return g;
}

Vec3 Geometry::neutralFoot(int leg) const
{
  const auto & m = mounts[leg];
  return {m.x + stance_radius * std::cos(m.yaw), m.y + stance_radius * std::sin(m.yaw), 0.0};
}

std::vector<std::string> Geometry::jointNames() const
{
  std::vector<std::string> out;
  out.reserve(kNumLegJoints);
  for (const auto & n : names) {
    out.push_back(n + "_coxa_joint");
    out.push_back(n + "_femur_joint");
    out.push_back(n + "_tibia_joint");
  }
  return out;
}

int Geometry::legIndex(const std::string & name) const
{
  for (int i = 0; i < kNumLegs; ++i) {
    if (names[i] == name) {
      return i;
    }
  }
  return -1;
}

Vec3 legFrameFromBase(const Geometry & g, int leg, const Vec3 & p_base)
{
  const auto & m = g.mounts[leg];
  return rotateZ({p_base.x - m.x, p_base.y - m.y, p_base.z}, -m.yaw);
}

Vec3 baseFromLegFrame(const Geometry & g, int leg, const Vec3 & p_leg)
{
  const auto & m = g.mounts[leg];
  Vec3 p = rotateZ(p_leg, m.yaw);
  return {p.x + m.x, p.y + m.y, p.z};
}

Vec3 forwardKinematicsLeg(const Geometry & g, const JointAngles & q)
{
  const double reach = g.coxa_length + g.femur_length * std::cos(q.femur) +
    g.tibia_length * std::sin(q.femur + q.tibia);
  const double z = g.femur_length * std::sin(q.femur) - g.tibia_length * std::cos(q.femur + q.tibia);
  return {reach * std::cos(q.coxa), reach * std::sin(q.coxa), z};
}

Vec3 forwardKinematics(const Geometry & g, int leg, const JointAngles & q)
{
  return baseFromLegFrame(g, leg, forwardKinematicsLeg(g, q));
}

IkResult inverseKinematics(const Geometry & g, int leg, const Vec3 & foot_base)
{
  IkResult res;
  const Vec3 p = legFrameFromBase(g, leg, foot_base);
  const double L2 = g.femur_length;
  const double L3 = g.tibia_length;

  const double coxa = std::atan2(p.y, p.x);
  const double r = std::hypot(p.x, p.y) - g.coxa_length;
  const double z = p.z;
  double c = (r * r + z * z - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
  res.reachable = c >= -1.0 && c <= 1.0;
  c = std::clamp(c, -1.0, 1.0);
  const double knee = -std::acos(c);  // knee above the femur-foot line
  const double femur = std::atan2(z, r) - std::atan2(L3 * std::sin(knee), L2 + L3 * std::cos(knee));
  const double tibia = knee + M_PI / 2.0;

  res.within_limits = g.coxa_limits.contains(coxa) && g.femur_limits.contains(femur) &&
    g.tibia_limits.contains(tibia);
  res.q.coxa = g.coxa_limits.clamp(coxa);
  res.q.femur = g.femur_limits.clamp(femur);
  res.q.tibia = g.tibia_limits.clamp(tibia);
  return res;
}

namespace
{
// R = Rz(yaw) * Ry(pitch) * Rx(roll), applied to v.
Vec3 rotateRPY(const Vec3 & v, double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // Rx
  Vec3 a{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
  // Ry
  Vec3 b{cp * a.x + sp * a.z, a.y, -sp * a.x + cp * a.z};
  // Rz
  return {cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z};
}

// R^T * v
Vec3 rotateRPYInverse(const Vec3 & v, double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Vec3 a{cy * v.x + sy * v.y, -sy * v.x + cy * v.y, v.z};
  Vec3 b{cp * a.x - sp * a.z, a.y, sp * a.x + cp * a.z};
  return {b.x, cr * b.y + sr * b.z, -sr * b.y + cr * b.z};
}
}  // namespace

Vec3 stanceToBase(const Vec3 & p_stance, const BodyPose & pose, double body_height)
{
  const Vec3 t{pose.x, pose.y, body_height + pose.z};
  return rotateRPYInverse(p_stance - t, pose.roll, pose.pitch, pose.yaw);
}

Vec3 baseToStance(const Vec3 & p_base, const BodyPose & pose, double body_height)
{
  const Vec3 t{pose.x, pose.y, body_height + pose.z};
  return rotateRPY(p_base, pose.roll, pose.pitch, pose.yaw) + t;
}

}  // namespace hexabot_locomotion
