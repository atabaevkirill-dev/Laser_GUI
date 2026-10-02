// Prints joint angles of a scripted walk as JSON.  The web simulation runs the
// same script with its JavaScript port of the gait and compares the result
// (sim/web/test/check_gait.mjs), so the browser shows the real robot gait.
#include <cstdio>
#include <string>
#include <vector>

#include "hexabot_locomotion/gait.hpp"
#include "hexabot_locomotion/geometry.hpp"

using namespace hexabot_locomotion;

struct Segment
{
  double duration;
  Twist2D cmd;
  GaitType gait;
};

int main()
{
  const Geometry g = Geometry::defaults();
  GaitGenerator gen(g);
  const std::vector<Segment> script = {
    {0.5, {0.0, 0.0, 0.0}, GaitType::kTripod},
    {2.0, {0.15, 0.0, 0.0}, GaitType::kTripod},
    {1.5, {0.05, 0.05, 0.4}, GaitType::kTripod},
    {1.5, {0.0, 0.0, 0.0}, GaitType::kTripod},
    {2.5, {0.08, -0.03, 0.0}, GaitType::kRipple},
    {2.0, {0.05, 0.0, -0.2}, GaitType::kWave},
    {2.0, {0.0, 0.0, 0.0}, GaitType::kWave},
  };
  const double dt = 0.01;
  const double height = g.body_height;
  std::printf("{\"dt\": %.3f, \"body_height\": %.4f, \"samples\": [\n", dt, height);
  bool first = true;
  int step = 0;
  for (const auto & seg : script) {
    gen.requestGait(seg.gait);
    gen.setCommand(seg.cmd);
    const int n = static_cast<int>(seg.duration / dt + 0.5);
    for (int k = 0; k < n; ++k, ++step) {
      gen.update(dt);
      if (step % 10 != 0) {continue;}
      std::printf("%s  {\"t\": %.2f, \"phase\": %.6f, \"q\": [", first ? "" : ",\n", step * dt,
        gen.phase());
      first = false;
      for (int i = 0; i < kNumLegs; ++i) {
        const auto r = inverseKinematics(g, i, stanceToBase(gen.feet()[i], BodyPose{}, height));
        std::printf("%s%.6f, %.6f, %.6f", i ? ", " : "", r.q.coxa, r.q.femur, r.q.tibia);
      }
      std::printf("]}");
    }
  }
  std::printf("\n]}\n");
  return 0;
}
