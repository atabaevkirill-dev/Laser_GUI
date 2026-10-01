#include "hexabot_locomotion/interpolator.hpp"

#include <algorithm>

#include "hexabot_locomotion/gait.hpp"

namespace hexabot_locomotion
{

void JointInterpolator::start(
  const std::vector<double> & from, const std::vector<double> & to, double duration)
{
  from_ = from;
  to_ = to;
  current_ = from;
  duration_ = std::max(duration, 1e-3);
  t_ = 0.0;
  active_ = true;
}

const std::vector<double> & JointInterpolator::update(double dt)
{
  if (!active_) {
    return current_;
  }
  t_ = std::min(t_ + dt, duration_);
  const double b = minJerk(t_ / duration_);
  for (size_t i = 0; i < current_.size() && i < to_.size(); ++i) {
    current_[i] = from_[i] + (to_[i] - from_[i]) * b;
  }
  if (t_ >= duration_) {
    active_ = false;
  }
  return current_;
}

}  // namespace hexabot_locomotion
