// Minimum-jerk interpolation between joint vectors (stand up / sit down).
#pragma once

#include <vector>

namespace hexabot_locomotion
{

class JointInterpolator
{
public:
  void start(const std::vector<double> & from, const std::vector<double> & to, double duration);
  /// Advances time and returns the current joint vector.
  const std::vector<double> & update(double dt);
  bool active() const {return active_;}
  const std::vector<double> & current() const {return current_;}

private:
  std::vector<double> from_;
  std::vector<double> to_;
  std::vector<double> current_;
  double duration_{0.0};
  double t_{0.0};
  bool active_{false};
};

}  // namespace hexabot_locomotion
