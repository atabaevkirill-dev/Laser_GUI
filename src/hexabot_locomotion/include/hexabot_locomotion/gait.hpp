// Phase-based gait generator with incremental foot placement.
//
// Feet live in the stance frame (see geometry.hpp).  Stance feet move with the
// ground (rigid motion opposite to the body twist); swing feet travel from
// their lift-off point to a Raibert-style touchdown point ahead of the neutral
// position, so the command may change at any time without jumps.
#pragma once

#include <array>
#include <string>

#include "hexabot_locomotion/geometry.hpp"
#include "hexabot_locomotion/types.hpp"

namespace hexabot_locomotion
{

enum class GaitType { kTripod, kRipple, kWave };

struct GaitPattern
{
  double duty{0.5};                          ///< stance fraction of a cycle
  std::array<double, kNumLegs> offset{};     ///< phase offsets, leg order lf lm lr rf rm rr
  double cycle_time{0.8};                    ///< seconds per cycle
};

std::string gaitName(GaitType type);
bool parseGait(const std::string & name, GaitType * out);

struct GaitConfig
{
  double max_stride{0.08};        ///< max foot travel during one stance phase (m)
  double linear_accel{0.6};       ///< command slew (m/s^2)
  double angular_accel{2.5};      ///< command slew (rad/s^2)
  double tripod_cycle{0.8};
  double ripple_cycle{1.05};
  double wave_cycle{1.4};
  double settle_tolerance{0.004}; ///< feet closer than this to neutral count as settled (m)
  double search_depth{0.04};      ///< how far a foot may reach below the expected ground (m)
  double search_speed{0.08};      ///< downward search speed in stance without contact (m/s)
};

GaitPattern gaitPattern(GaitType type, const GaitConfig & cfg = {});

class GaitGenerator
{
public:
  explicit GaitGenerator(const Geometry & geometry, const GaitConfig & config = {});

  /// Desired body twist (body frame).  It is limited to the stride and slewed.
  void setCommand(const Twist2D & cmd);
  /// Gait changes are applied once the robot has come to rest.
  void requestGait(GaitType type);
  void setStepHeight(double h);
  /// Expected ground height under a foot (stance frame z, metres).
  void setGroundHeight(int leg, double z);
  /// Contact sensing (servo load / foot switch): enables adaptive touchdown.
  void setContactSensing(bool enabled);
  void setFootContact(int leg, bool contact);
  /// Put all feet at their neutral positions and stop.
  void reset();

  void update(double dt);

  const std::array<Vec3, kNumLegs> & feet() const {return feet_;}
  std::array<bool, kNumLegs> inStance() const;
  double phase() const {return phase_;}
  bool active() const {return active_;}
  GaitType gait() const {return gait_;}
  GaitType requestedGait() const {return requested_gait_;}
  const GaitPattern & pattern() const {return pattern_;}
  /// Command currently executed (after limits and slew).
  const Twist2D & command() const {return command_;}
  double stepHeight() const {return step_height_;}
  /// Mean expected ground height under the feet; the body follows it.
  double meanGroundHeight() const;
  /// Largest twist components the current gait can execute.
  Twist2D maxCommand() const;
  /// Scales a twist so no foot exceeds the max stride.
  Twist2D limitCommand(const Twist2D & cmd) const;

private:
  Vec3 strideVector(int leg, const Twist2D & cmd) const;
  Vec3 swingTarget(int leg) const;
  bool settled() const;
  bool footSettled(int leg) const;

  Geometry geo_;
  GaitConfig cfg_;
  GaitType gait_{GaitType::kTripod};
  GaitType requested_gait_{GaitType::kTripod};
  GaitPattern pattern_{};
  Twist2D target_{};
  Twist2D command_{};
  double step_height_{0.04};
  double phase_{0.0};
  bool active_{false};
  bool contact_sensing_{false};
  std::array<Vec3, kNumLegs> feet_{};
  std::array<Vec3, kNumLegs> liftoff_{};
  std::array<double, kNumLegs> ground_{};
  std::array<double, kNumLegs> expected_ground_{};
  std::array<bool, kNumLegs> was_stance_{};
  std::array<bool, kNumLegs> contact_{};
  std::array<bool, kNumLegs> touched_{};
  std::array<bool, kNumLegs> hold_{};       ///< planted until its next stance (restart mid-swing)
  std::array<double, kNumLegs> swing_s0_{}; ///< swing fraction already elapsed at (re)start
};

/// Swing height profile, 0 at both ends and 1 in the middle (smooth).
double swingHeightProfile(double s);
/// Minimum-jerk blend 0..1.
double minJerk(double s);

}  // namespace hexabot_locomotion
