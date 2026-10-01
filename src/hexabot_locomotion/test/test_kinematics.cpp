#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "hexabot_locomotion/geometry.hpp"

using namespace hexabot_locomotion;

TEST(Kinematics, ZeroAnglesGiveRightAngleKnee)
{
  const Geometry g = Geometry::defaults();
  const Vec3 p = forwardKinematicsLeg(g, {0.0, 0.0, 0.0});
  EXPECT_NEAR(p.x, g.coxa_length + g.femur_length, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.z, -g.tibia_length, 1e-12);
}

TEST(Kinematics, NeutralStanceIsReachable)
{
  const Geometry g = Geometry::defaults();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 foot = stanceToBase(g.neutralFoot(leg), BodyPose{}, g.body_height);
    const IkResult r = inverseKinematics(g, leg, foot);
    EXPECT_TRUE(r.reachable) << g.names[leg];
    EXPECT_TRUE(r.within_limits) << g.names[leg];
    EXPECT_NEAR(r.q.coxa, 0.0, 1e-9);
    // Femur lifted, tibia close to vertical (see cad/params.py stance).
    EXPECT_GT(r.q.femur, 0.2);
    EXPECT_NEAR(r.q.femur + r.q.tibia, 0.0, 0.1);
  }
}

TEST(Kinematics, InverseOfForwardRoundTrip)
{
  const Geometry g = Geometry::defaults();
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uc(g.coxa_limits.min, g.coxa_limits.max);
  std::uniform_real_distribution<double> uf(-0.5, 1.4);
  std::uniform_real_distribution<double> ut(-1.0, 0.8);
  int checked = 0;
  for (int k = 0; k < 2000; ++k) {
    const JointAngles q{uc(rng), uf(rng), ut(rng)};
    // Only the knee-up branch is produced by the IK.
    const double knee = q.tibia - M_PI / 2.0;
    if (knee > -1e-3) {continue;}
    // The foot must be outside the femur axis, otherwise the coxa solution
    // flips by 180 degrees (equally valid, but not this configuration).
    const double reach = g.coxa_length + g.femur_length * std::cos(q.femur) +
      g.tibia_length * std::sin(q.femur + q.tibia);
    if (reach < g.coxa_length + 0.005) {continue;}
    const int leg = k % kNumLegs;
    const Vec3 foot = forwardKinematics(g, leg, q);
    const IkResult r = inverseKinematics(g, leg, foot);
    ASSERT_TRUE(r.reachable);
    EXPECT_NEAR(r.q.coxa, q.coxa, 1e-6);
    EXPECT_NEAR(r.q.femur, q.femur, 1e-6);
    EXPECT_NEAR(r.q.tibia, q.tibia, 1e-6);
    const Vec3 back = forwardKinematics(g, leg, r.q);
    EXPECT_NEAR((back - foot).norm(), 0.0, 1e-9);
    ++checked;
  }
  EXPECT_GT(checked, 500);
}

TEST(Kinematics, UnreachableIsFlagged)
{
  const Geometry g = Geometry::defaults();
  const Vec3 far = baseFromLegFrame(g, 0, {1.0, 0.0, -0.1});
  EXPECT_FALSE(inverseKinematics(g, 0, far).reachable);
}

TEST(Kinematics, StanceBaseTransformRoundTrip)
{
  const BodyPose pose{0.01, -0.02, 0.015, 0.1, -0.15, 0.2};
  const Vec3 p{0.2, -0.1, 0.0};
  const Vec3 b = stanceToBase(p, pose, 0.1);
  const Vec3 back = baseToStance(b, pose, 0.1);
  EXPECT_NEAR((back - p).norm(), 0.0, 1e-12);
  // Raising the body lowers the feet in base_link.
  EXPECT_NEAR(stanceToBase(p, BodyPose{}, 0.1).z, -0.1, 1e-12);
}

TEST(Kinematics, JointNamesInControllerOrder)
{
  const auto names = Geometry::defaults().jointNames();
  ASSERT_EQ(names.size(), static_cast<size_t>(kNumLegJoints));
  EXPECT_EQ(names[0], "lf_coxa_joint");
  EXPECT_EQ(names[5], "lm_tibia_joint");
  EXPECT_EQ(names[17], "rr_tibia_joint");
}
