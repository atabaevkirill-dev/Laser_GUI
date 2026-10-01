#include <gtest/gtest.h>

#include <cmath>

#include "hexabot_locomotion/gait.hpp"
#include "hexabot_locomotion/geometry.hpp"

using namespace hexabot_locomotion;

namespace
{
int stanceCount(const std::array<bool, kNumLegs> & s)
{
  int n = 0;
  for (bool b : s) {n += b ? 1 : 0;}
  return n;
}

// Signed area test: is (px, py) inside the convex hull of the stance feet?
bool insideSupport(const std::array<Vec3, kNumLegs> & feet, const std::array<bool, kNumLegs> & st,
  double px, double py)
{
  std::vector<Vec3> pts;
  for (int i = 0; i < kNumLegs; ++i) {
    if (st[i]) {pts.push_back(feet[i]);}
  }
  if (pts.size() < 3) {return false;}
  // Sort by angle around the centroid.
  double cx = 0, cy = 0;
  for (auto & p : pts) {cx += p.x; cy += p.y;}
  cx /= pts.size();
  cy /= pts.size();
  std::sort(pts.begin(), pts.end(), [&](const Vec3 & a, const Vec3 & b) {
      return std::atan2(a.y - cy, a.x - cx) < std::atan2(b.y - cy, b.x - cx);
    });
  for (size_t i = 0; i < pts.size(); ++i) {
    const Vec3 & a = pts[i];
    const Vec3 & b = pts[(i + 1) % pts.size()];
    if ((b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x) < 0) {return false;}
  }
  return true;
}
}  // namespace

TEST(Gait, PatternsKeepEnoughLegsOnTheGround)
{
  for (GaitType t : {GaitType::kTripod, GaitType::kRipple, GaitType::kWave}) {
    const GaitPattern p = gaitPattern(t);
    const int expected = t == GaitType::kTripod ? 3 : (t == GaitType::kRipple ? 4 : 5);
    for (int k = 0; k < 600; ++k) {
      const double phase = (k + 0.5) / 600.0;
      int n = 0;
      for (int i = 0; i < kNumLegs; ++i) {
        double psi = std::fmod(phase + p.offset[i], 1.0);
        n += psi < p.duty ? 1 : 0;
      }
      EXPECT_EQ(n, expected) << gaitName(t) << " phase " << phase;
    }
  }
}

TEST(Gait, StandsStillWithoutCommand)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  for (int k = 0; k < 200; ++k) {gen.update(0.01);}
  EXPECT_FALSE(gen.active());
  for (int i = 0; i < kNumLegs; ++i) {
    const Vec3 n = g.neutralFoot(i);
    EXPECT_NEAR((gen.feet()[i] - n).norm(), 0.0, 1e-12);
  }
}

TEST(Gait, WalksForwardStablyAndStops)
{
  const Geometry g = Geometry::defaults();
  for (GaitType t : {GaitType::kTripod, GaitType::kRipple, GaitType::kWave}) {
    GaitGenerator gen(g);
    gen.requestGait(t);
    gen.update(0.01);   // applies the gait while at rest
    ASSERT_EQ(gen.gait(), t);
    gen.setCommand({0.5, 0.0, 0.0});   // more than possible: must be limited
    double distance = 0.0;
    double max_height = 0.0;
    for (int k = 0; k < 600; ++k) {
      gen.update(0.01);
      distance += gen.command().vx * 0.01;
      const auto st = gen.inStance();
      EXPECT_GE(stanceCount(st), 3);
      EXPECT_TRUE(insideSupport(gen.feet(), st, 0.0, 0.0)) << gaitName(t) << " step " << k;
      for (int i = 0; i < kNumLegs; ++i) {
        max_height = std::max(max_height, gen.feet()[i].z);
        // Feet never leave the workspace.
        const IkResult r = inverseKinematics(
          g, i, stanceToBase(gen.feet()[i], BodyPose{}, g.body_height));
        EXPECT_TRUE(r.reachable && r.within_limits) << gaitName(t) << " leg " << i;
        // Stance feet stay on the ground.
        if (st[i]) {EXPECT_NEAR(gen.feet()[i].z, 0.0, 1e-9);}
      }
    }
    EXPECT_LE(gen.command().vx, gen.maxCommand().vx + 1e-9);
    EXPECT_GT(distance, 0.2) << gaitName(t);
    EXPECT_NEAR(max_height, gen.stepHeight(), 0.003);

    gen.setCommand({});
    for (int k = 0; k < 400; ++k) {gen.update(0.01);}
    EXPECT_FALSE(gen.active()) << gaitName(t);
    for (int i = 0; i < kNumLegs; ++i) {
      EXPECT_LT((gen.feet()[i] - g.neutralFoot(i)).normXY(), 0.005);
    }
  }
}

TEST(Gait, StanceFeetMoveWithTheGround)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  gen.setCommand({0.1, 0.0, 0.3});
  for (int k = 0; k < 300; ++k) {gen.update(0.01);}
  const auto before = gen.feet();
  const auto st = gen.inStance();
  const Twist2D c = gen.command();
  gen.update(0.01);
  const auto after = gen.feet();
  const auto st2 = gen.inStance();
  for (int i = 0; i < kNumLegs; ++i) {
    if (!(st[i] && st2[i])) {continue;}
    // Ground point velocity in the body frame: -(v + w x p).
    const double ex = -(c.vx - c.wz * before[i].y) * 0.01;
    const double ey = -(c.vy + c.wz * before[i].x) * 0.01;
    EXPECT_NEAR(after[i].x - before[i].x, ex, 2e-5);
    EXPECT_NEAR(after[i].y - before[i].y, ey, 2e-5);
  }
}

TEST(Gait, GaitChangeWaitsForRest)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  gen.setCommand({0.1, 0.0, 0.0});
  for (int k = 0; k < 150; ++k) {gen.update(0.01);}
  ASSERT_TRUE(gen.active());
  gen.requestGait(GaitType::kWave);
  gen.update(0.01);
  EXPECT_EQ(gen.gait(), GaitType::kTripod);
  int k = 0;
  for (; k < 500 && gen.gait() != GaitType::kWave; ++k) {gen.update(0.01);}
  EXPECT_EQ(gen.gait(), GaitType::kWave);
  // ...and it keeps walking afterwards.
  for (int j = 0; j < 100; ++j) {gen.update(0.01);}
  EXPECT_TRUE(gen.active());
}

TEST(Gait, RestartMidSwingHasNoJumps)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  for (int trial = 0; trial < 20; ++trial) {
    gen.setCommand({0.12, 0.02 * (trial % 3), 0.1});
    for (int k = 0; k < 37 + trial * 7; ++k) {gen.update(0.01);}
    gen.setCommand({});
    for (int k = 0; k < 13 * trial % 250; ++k) {gen.update(0.01);}
    auto prev = gen.feet();
    gen.setCommand({0.15, -0.03, 0.0});
    for (int k = 0; k < 120; ++k) {
      gen.update(0.01);
      for (int i = 0; i < kNumLegs; ++i) {
        // 2 cm per 10 ms is far above any legitimate foot speed.
        EXPECT_LT((gen.feet()[i] - prev[i]).norm(), 0.02) << "trial " << trial;
      }
      prev = gen.feet();
    }
  }
}

TEST(Gait, StepHeightIsClamped)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  gen.setStepHeight(1.0);
  EXPECT_DOUBLE_EQ(gen.stepHeight(), g.step_height_max);
  gen.setStepHeight(0.0);
  EXPECT_DOUBLE_EQ(gen.stepHeight(), 0.01);
}

TEST(Gait, ContactSensingStopsEarlyOnHighGround)
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  gen.setContactSensing(true);
  gen.setCommand({0.1, 0.0, 0.0});
  // Leg 0 steps onto a 3 cm block: contact whenever its foot is at or below 3 cm.
  double landed = -1.0;
  for (int k = 0; k < 400; ++k) {
    for (int i = 0; i < kNumLegs; ++i) {
      const double ground = i == 0 ? 0.03 : 0.0;
      gen.setFootContact(i, gen.feet()[i].z <= ground + 1e-4);
    }
    gen.update(0.01);
    if (gen.inStance()[0] && k > 100) {landed = gen.feet()[0].z;}
  }
  EXPECT_NEAR(landed, 0.03, 0.006);
}
