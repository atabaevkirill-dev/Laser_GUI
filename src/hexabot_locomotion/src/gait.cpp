#include "hexabot_locomotion/gait.hpp"

#include <algorithm>
#include <cmath>

namespace hexabot_locomotion
{

std::string gaitName(GaitType type)
{
  switch (type) {
    case GaitType::kTripod: return "tripod";
    case GaitType::kRipple: return "ripple";
    case GaitType::kWave: return "wave";
  }
  return "tripod";
}

bool parseGait(const std::string & name, GaitType * out)
{
  if (name == "tripod") {
    *out = GaitType::kTripod;
  } else if (name == "ripple") {
    *out = GaitType::kRipple;
  } else if (name == "wave") {
    *out = GaitType::kWave;
  } else {
    return false;
  }
  return true;
}

GaitPattern gaitPattern(GaitType type, const GaitConfig & cfg)
{
  // A leg swings while phase is in [duty - offset, 1 - offset) (mod 1).
  // Order lf lm lr rf rm rr.
  GaitPattern p;
  switch (type) {
    case GaitType::kTripod:
      // {lf, lr, rm} and {lm, rf, rr} alternate.
      p.duty = 0.5;
      p.offset = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
      p.cycle_time = cfg.tripod_cycle;
      break;
    case GaitType::kRipple:
      // Back-to-front waves on each side, sides half a cycle apart.
      p.duty = 2.0 / 3.0;
      p.offset = {0.0, 1.0 / 3.0, 2.0 / 3.0, 0.5, 5.0 / 6.0, 1.0 / 6.0};
      p.cycle_time = cfg.ripple_cycle;
      break;
    case GaitType::kWave:
      // One leg at a time: lr lm lf rr rm rf.
      p.duty = 5.0 / 6.0;
      p.offset = {0.5, 2.0 / 3.0, 5.0 / 6.0, 0.0, 1.0 / 6.0, 1.0 / 3.0};
      p.cycle_time = cfg.wave_cycle;
      break;
  }
  return p;
}

double swingHeightProfile(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return 0.5 * (1.0 - std::cos(2.0 * M_PI * s));
}

double minJerk(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

namespace
{
double frac(double v)
{
  v = std::fmod(v, 1.0);
  return v < 0.0 ? v + 1.0 : v;
}

double slew(double current, double target, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}
}  // namespace

GaitGenerator::GaitGenerator(const Geometry & geometry, const GaitConfig & config)
: geo_(geometry), cfg_(config)
{
  pattern_ = gaitPattern(gait_, cfg_);
  step_height_ = geo_.step_height;
  reset();
}

void GaitGenerator::reset()
{
  for (int i = 0; i < kNumLegs; ++i) {
    feet_[i] = geo_.neutralFoot(i);
    feet_[i].z = ground_[i];
    liftoff_[i] = feet_[i];
    was_stance_[i] = true;
    touched_[i] = false;
    hold_[i] = false;
    swing_s0_[i] = 0.0;
  }
  command_ = {};
  active_ = false;
}

void GaitGenerator::setCommand(const Twist2D & cmd) {target_ = cmd;}

void GaitGenerator::requestGait(GaitType type) {requested_gait_ = type;}

void GaitGenerator::setStepHeight(double h)
{
  step_height_ = std::clamp(h, 0.01, geo_.step_height_max);
}

void GaitGenerator::setGroundHeight(int leg, double z)
{
  expected_ground_[leg] = z;
  if (!contact_sensing_) {
    ground_[leg] = z;
  }
}

void GaitGenerator::setContactSensing(bool enabled) {contact_sensing_ = enabled;}

void GaitGenerator::setFootContact(int leg, bool contact) {contact_[leg] = contact;}

std::array<bool, kNumLegs> GaitGenerator::inStance() const
{
  std::array<bool, kNumLegs> out{};
  for (int i = 0; i < kNumLegs; ++i) {
    out[i] = !active_ || hold_[i] || frac(phase_ + pattern_.offset[i]) < pattern_.duty;
  }
  return out;
}

double GaitGenerator::meanGroundHeight() const
{
  double sum = 0.0;
  for (double g : ground_) {sum += g;}
  return sum / kNumLegs;
}

Vec3 GaitGenerator::strideVector(int leg, const Twist2D & cmd) const
{
  const Vec3 p = geo_.neutralFoot(leg);
  const double t_stance = pattern_.duty * pattern_.cycle_time;
  // Velocity of the body point above the foot: v + w x p.
  return {(cmd.vx - cmd.wz * p.y) * t_stance, (cmd.vy + cmd.wz * p.x) * t_stance, 0.0};
}

Twist2D GaitGenerator::maxCommand() const
{
  const double t_stance = pattern_.duty * pattern_.cycle_time;
  double r_max = 0.0;
  for (int i = 0; i < kNumLegs; ++i) {
    r_max = std::max(r_max, geo_.neutralFoot(i).normXY());
  }
  const double v = cfg_.max_stride / t_stance;
  return {v, v, v / r_max};
}

Twist2D GaitGenerator::limitCommand(const Twist2D & cmd) const
{
  const Twist2D lim = maxCommand();
  Twist2D c{std::clamp(cmd.vx, -lim.vx, lim.vx), std::clamp(cmd.vy, -lim.vy, lim.vy),
    std::clamp(cmd.wz, -lim.wz, lim.wz)};
  double worst = 0.0;
  for (int i = 0; i < kNumLegs; ++i) {
    worst = std::max(worst, strideVector(i, c).normXY());
  }
  if (worst > cfg_.max_stride) {
    const double k = cfg_.max_stride / worst;
    c.vx *= k;
    c.vy *= k;
    c.wz *= k;
  }
  return c;
}

Vec3 GaitGenerator::swingTarget(int leg) const
{
  Vec3 t = geo_.neutralFoot(leg) + strideVector(leg, command_) * 0.5;
  t.z = ground_[leg];
  return t;
}

bool GaitGenerator::footSettled(int leg) const
{
  const Vec3 n = geo_.neutralFoot(leg);
  return std::hypot(feet_[leg].x - n.x, feet_[leg].y - n.y) <= cfg_.settle_tolerance &&
         std::abs(feet_[leg].z - ground_[leg]) <= cfg_.settle_tolerance;
}

bool GaitGenerator::settled() const
{
  for (int i = 0; i < kNumLegs; ++i) {
    if (!footSettled(i)) {
      return false;
    }
  }
  return true;
}

void GaitGenerator::update(double dt)
{
  if (dt <= 0.0) {
    return;
  }
  const bool gait_change = requested_gait_ != gait_;
  const Twist2D target = gait_change ? Twist2D{} : limitCommand(target_);
  command_.vx = slew(command_.vx, target.vx, cfg_.linear_accel * dt);
  command_.vy = slew(command_.vy, target.vy, cfg_.linear_accel * dt);
  command_.wz = slew(command_.wz, target.wz, cfg_.angular_accel * dt);
  const bool want_motion = !command_.isZero(1e-4) || !target.isZero(1e-4);

  if (!active_) {
    if (gait_change) {
      gait_ = requested_gait_;
      pattern_ = gaitPattern(gait_, cfg_);
      return;
    }
    if (!want_motion) {
      // Standing: feet stay planted, still follow the expected ground height.
      for (int i = 0; i < kNumLegs; ++i) {
        if (!contact_sensing_) {
          feet_[i].z = ground_[i];
        }
      }
      return;
    }
    active_ = true;
    for (int i = 0; i < kNumLegs; ++i) {
      const double psi = frac(phase_ + pattern_.offset[i]);
      liftoff_[i] = feet_[i];
      touched_[i] = false;
      swing_s0_[i] = 0.0;
      hold_[i] = false;
      was_stance_[i] = psi < pattern_.duty;
      if (!was_stance_[i]) {
        // Restarting in the middle of a swing: finish it in the remaining
        // time, or keep the foot planted if too little time is left.
        const double s = (psi - pattern_.duty) / (1.0 - pattern_.duty);
        if (1.0 - s < 0.3) {
          hold_[i] = true;
        } else {
          swing_s0_[i] = s;
        }
      }
    }
  }

  phase_ = frac(phase_ + dt / pattern_.cycle_time);
  const double duty = pattern_.duty;

  for (int i = 0; i < kNumLegs; ++i) {
    const double psi = frac(phase_ + pattern_.offset[i]);
    const bool stance = psi < duty;
    if (stance) {
      hold_[i] = false;
    } else if (was_stance_[i] && !want_motion && footSettled(i)) {
      // Coming to rest: a foot already at its neutral spot skips the swing.
      hold_[i] = true;
    }
    Vec3 & f = feet_[i];
    if (stance || hold_[i]) {
      if (!was_stance_[i]) {
        // Touchdown.  With contact sensing a foot that touched early keeps
        // that height; one that did not touch keeps searching downwards.
        if (contact_sensing_ && !touched_[i] && !contact_[i]) {
          ground_[i] = f.z;
        }
        touched_[i] = false;
      }
      if (contact_sensing_ && !contact_[i]) {
        ground_[i] = std::max(ground_[i] - cfg_.search_speed * dt,
            expected_ground_[i] - cfg_.search_depth);
      }
      const Vec3 moved = rotateZ({f.x, f.y, 0.0}, -command_.wz * dt);
      f.x = moved.x - command_.vx * dt;
      f.y = moved.y - command_.vy * dt;
      f.z = ground_[i];
    } else {
      if (was_stance_[i]) {
        liftoff_[i] = f;
        touched_[i] = false;
        swing_s0_[i] = 0.0;
        if (contact_sensing_) {
          ground_[i] = expected_ground_[i];
        }
      }
      const double s_raw = (psi - duty) / (1.0 - duty);
      const double s = std::clamp((s_raw - swing_s0_[i]) / (1.0 - swing_s0_[i]), 0.0, 1.0);
      if (contact_sensing_ && s > 0.5 && contact_[i] && !touched_[i]) {
        touched_[i] = true;   // early touchdown on something higher than expected
        ground_[i] = f.z;
      }
      const Vec3 target = swingTarget(i);
      const double b = minJerk(s);
      f.x = liftoff_[i].x + (target.x - liftoff_[i].x) * b;
      f.y = liftoff_[i].y + (target.y - liftoff_[i].y) * b;
      if (touched_[i]) {
        f.z = ground_[i];
      } else {
        const double base = liftoff_[i].z + (ground_[i] - liftoff_[i].z) * b;
        f.z = base + step_height_ * swingHeightProfile(s);
      }
    }
    was_stance_[i] = stance;
  }

  if (!want_motion && settled()) {
    active_ = false;
    for (int i = 0; i < kNumLegs; ++i) {
      feet_[i].z = ground_[i];
      was_stance_[i] = true;
    }
  }
}

}  // namespace hexabot_locomotion
