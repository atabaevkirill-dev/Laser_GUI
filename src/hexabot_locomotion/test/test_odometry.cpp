#include <gtest/gtest.h>

#include <cmath>

#include "hexabot_locomotion/gait.hpp"
#include "hexabot_locomotion/geometry.hpp"
#include "hexabot_locomotion/odometry.hpp"

using namespace hexabot_locomotion;

TEST(Odometry, RecoversRigidMotion)
{
  const Geometry g = Geometry::defaults();
  std::array<Vec3, kNumLegs> prev{}, curr{};
  const Pose2D motion{0.012, -0.004, 0.03};
  const double c = std::cos(motion.yaw), s = std::sin(motion.yaw);
  for (int i = 0; i < kNumLegs; ++i) {
    prev[i] = g.neutralFoot(i);
    // a = R b + d  ->  b = R^T (a - d)
    const double dx = prev[i].x - motion.x, dy = prev[i].y - motion.y;
    curr[i] = {c * dx + s * dy, -s * dx + c * dy, 0.0};
  }
  std::array<bool, kNumLegs> st{true, false, true, false, true, false};
  Pose2D d;
  ASSERT_TRUE(estimateBodyMotion(prev, curr, st, &d));
  EXPECT_NEAR(d.x, motion.x, 1e-12);
  EXPECT_NEAR(d.y, motion.y, 1e-12);
  EXPECT_NEAR(d.yaw, motion.yaw, 1e-12);
  st = {true, false, false, false, false, false};
  EXPECT_FALSE(estimateBodyMotion(prev, curr, st, &d));
}

TEST(Odometry, MatchesTheCommandedWalk)
{
  // Feed the generated feet into the odometry: the estimate must match the
  // integral of the executed command.
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  LegOdometry odom;
  Pose2D truth{};
  gen.setCommand({0.1, 0.03, 0.25});
  for (int k = 0; k < 800; ++k) {
    gen.update(0.01);
    const Twist2D c = gen.command();
    truth = integratePose(truth, {c.vx * 0.01, c.vy * 0.01, c.wz * 0.01});
    odom.update(gen.feet(), gen.inStance());
  }
  EXPECT_NEAR(odom.pose().x, truth.x, 0.01);
  EXPECT_NEAR(odom.pose().y, truth.y, 0.01);
  EXPECT_NEAR(wrapAngle(odom.pose().yaw - truth.yaw), 0.0, 0.02);
  EXPECT_GT(std::hypot(truth.x, truth.y), 0.3);
}
