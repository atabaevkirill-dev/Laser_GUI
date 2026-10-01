// Basic types shared by the Hexabot locomotion library (no ROS dependencies).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace hexabot_locomotion
{

constexpr int kNumLegs = 6;
constexpr int kJointsPerLeg = 3;
constexpr int kNumLegJoints = kNumLegs * kJointsPerLeg;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  Vec3 operator-(const Vec3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  Vec3 operator*(double s) const {return {x * s, y * s, z * s};}
  Vec3 & operator+=(const Vec3 & o)
  {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  double norm() const {return std::sqrt(x * x + y * y + z * z);}
  double normXY() const {return std::hypot(x, y);}
};

struct JointAngles
{
  double coxa{0.0};
  double femur{0.0};
  double tibia{0.0};
};

struct Limits
{
  double min{-M_PI};
  double max{M_PI};

  double clamp(double v) const {return std::clamp(v, min, max);}
  bool contains(double v, double eps = 1e-9) const {return v >= min - eps && v <= max + eps;}
};

/// Coxa axis position in base_link and the leg's neutral direction.
struct LegMount
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Body offset relative to the neutral stance.
struct BodyPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

/// Planar velocity command in the body frame.
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  bool isZero(double eps = 1e-6) const
  {
    return std::abs(vx) < eps && std::abs(vy) < eps && std::abs(wz) < eps;
  }
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

/// Rotate a vector about Z.
inline Vec3 rotateZ(const Vec3 & v, double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

}  // namespace hexabot_locomotion
