// ROS 2 node: body twist + pose -> gait -> leg IK -> joint position commands.
//
// Subscribes  cmd_vel (Twist or TwistStamped), body_pose, joint_states, imu,
//             terrain/info, step_height
// Publishes   leg_controller/commands (Float64MultiArray, 18 joints),
//             odom (+ TF odom -> base_link), locomotion/state
// Services    ~/set_gait, ~/set_mode
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hexabot_interfaces/msg/body_pose.hpp"
#include "hexabot_interfaces/msg/locomotion_state.hpp"
#include "hexabot_interfaces/msg/terrain_info.hpp"
#include "hexabot_interfaces/srv/set_gait.hpp"
#include "hexabot_interfaces/srv/set_mode.hpp"
#include "hexabot_locomotion/gait.hpp"
#include "hexabot_locomotion/geometry.hpp"
#include "hexabot_locomotion/interpolator.hpp"
#include "hexabot_locomotion/odometry.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace hexabot_locomotion
{

using namespace std::chrono_literals;
using SetGait = hexabot_interfaces::srv::SetGait;
using SetMode = hexabot_interfaces::srv::SetMode;
using SetHwState = controller_manager_msgs::srv::SetHardwareComponentState;

enum class Mode { kUnknown, kStandingUp, kStanding, kSittingDown, kSitting, kFallen, kRelaxed };

const char * modeName(Mode m)
{
  switch (m) {
    case Mode::kUnknown: return "unknown";
    case Mode::kStandingUp: return "standing_up";
    case Mode::kStanding: return "standing";
    case Mode::kSittingDown: return "sitting_down";
    case Mode::kSitting: return "sitting";
    case Mode::kFallen: return "fallen";
    case Mode::kRelaxed: return "relaxed";
  }
  return "unknown";
}

class LocomotionNode : public rclcpp::Node
{
public:
  LocomotionNode()
  : Node("locomotion")
  {
    geo_ = loadGeometry();

    rate_ = declare_parameter("rate", 100.0);
    GaitConfig cfg;
    cfg.max_stride = declare_parameter("max_stride", cfg.max_stride);
    cfg.linear_accel = declare_parameter("linear_accel", cfg.linear_accel);
    cfg.angular_accel = declare_parameter("angular_accel", cfg.angular_accel);
    cfg.tripod_cycle = declare_parameter("tripod_cycle", cfg.tripod_cycle);
    cfg.ripple_cycle = declare_parameter("ripple_cycle", cfg.ripple_cycle);
    cfg.wave_cycle = declare_parameter("wave_cycle", cfg.wave_cycle);
    gait_ = std::make_unique<GaitGenerator>(geo_, cfg);

    GaitType g = GaitType::kTripod;
    const auto gait_name = declare_parameter("gait", std::string("tripod"));
    if (!parseGait(gait_name, &g)) {
      RCLCPP_WARN(get_logger(), "unknown gait '%s', using tripod", gait_name.c_str());
    }
    gait_->requestGait(g);
    gait_->setStepHeight(declare_parameter("step_height", geo_.step_height));

    nominal_height_ = declare_parameter("body_height", geo_.body_height);
    crouch_height_ = declare_parameter("crouch_height", geo_.body_height_range.min + 0.01);
    sit_height_ = declare_parameter("sit_height", 0.03);
    height_rate_ = declare_parameter("body_height_rate", 0.05);
    transition_time_ = declare_parameter("transition_time", 1.5);
    start_mode_ = declare_parameter("start_mode", std::string("wait"));
    cmd_timeout_ = declare_parameter("command_timeout", 0.5);
    publish_tf_ = declare_parameter("publish_tf", true);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    footprint_frame_ = declare_parameter("footprint_frame", std::string("base_footprint"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    odom_from_joints_ = declare_parameter("odom_from_joints", true);
    leveling_ = declare_parameter("imu_leveling", false);
    leveling_gain_ = declare_parameter("leveling_gain", 0.8);
    leveling_limit_ = declare_parameter("leveling_limit", 0.2);
    fall_angle_ = declare_parameter("fall_angle", 0.9);
    auto_terrain_ = declare_parameter("auto_terrain", true);
    contact_threshold_ = declare_parameter("contact_effort_threshold", 0.0);
    hw_component_ = declare_parameter("hardware_component", std::string("HexabotSystem"));
    const bool stamped = declare_parameter("use_stamped_cmd_vel", false);

    if (contact_threshold_ > 0.0) {
      gait_->setContactSensing(true);
    }

    joint_names_ = geo_.jointNames();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = static_cast<int>(i);
    }
    measured_.assign(kNumLegJoints, 0.0);
    effort_.assign(kNumLegJoints, 0.0);
    body_height_ = sit_height_;   // assume we start resting on the ground
    height_target_ = nominal_height_;

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("leg_controller/commands", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    state_pub_ = create_publisher<hexabot_interfaces::msg::LocomotionState>("locomotion/state", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (stamped) {
      cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {onTwist(m->twist);});
    } else {
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {onTwist(*m);});
    }
    pose_sub_ = create_subscription<hexabot_interfaces::msg::BodyPose>(
      "body_pose", 10, [this](hexabot_interfaces::msg::BodyPose::ConstSharedPtr m) {
        user_pose_ = {m->x, m->y, m->z, m->roll, m->pitch, m->yaw};
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {onJointStates(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});
    step_sub_ = create_subscription<std_msgs::msg::Float64>(
      "step_height", 10, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
        gait_->setStepHeight(m->data);
      });
    terrain_sub_ = create_subscription<hexabot_interfaces::msg::TerrainInfo>(
      "terrain/info", 10, [this](hexabot_interfaces::msg::TerrainInfo::ConstSharedPtr m) {
        onTerrain(*m);
      });

    gait_srv_ = create_service<SetGait>(
      "~/set_gait", [this](SetGait::Request::SharedPtr req, SetGait::Response::SharedPtr res) {
        GaitType t;
        if (!parseGait(req->gait, &t)) {
          res->success = false;
          res->message = "unknown gait (tripod | ripple | wave)";
          return;
        }
        gait_->requestGait(t);
        res->success = true;
        res->message = "gait '" + req->gait + "' requested";
      });
    mode_srv_ = create_service<SetMode>(
      "~/set_mode", [this](SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res) {
        onSetMode(*req, *res);
      });
    hw_client_ = create_client<SetHwState>("controller_manager/set_hardware_component_state");

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "locomotion ready (%s gait, %.0f Hz, start_mode=%s)",
      gaitName(gait_->requestedGait()).c_str(), rate_, start_mode_.c_str());
  }

private:
  Geometry loadGeometry()
  {
    Geometry g = Geometry::defaults();
    auto vec = [this](const std::string & name, const std::vector<double> & def) {
        return declare_parameter("geometry." + name, def);
      };
    auto num = [this](const std::string & name, double def) {
        return declare_parameter("geometry." + name, def);
      };
    std::vector<double> mx, my, myaw;
    for (const auto & m : g.mounts) {
      mx.push_back(m.x);
      my.push_back(m.y);
      myaw.push_back(m.yaw);
    }
    mx = vec("mount_x", mx);
    my = vec("mount_y", my);
    myaw = vec("mount_yaw", myaw);
    if (mx.size() != kNumLegs || my.size() != kNumLegs || myaw.size() != kNumLegs) {
      throw std::runtime_error("geometry.mount_* must have 6 entries");
    }
    for (int i = 0; i < kNumLegs; ++i) {
      g.mounts[i] = {mx[i], my[i], myaw[i]};
    }
    const auto names = declare_parameter(
      "geometry.leg_names", std::vector<std::string>(g.names.begin(), g.names.end()));
    for (int i = 0; i < kNumLegs && i < static_cast<int>(names.size()); ++i) {
      g.names[i] = names[i];
    }
    g.coxa_length = num("coxa_length", g.coxa_length);
    g.femur_length = num("femur_length", g.femur_length);
    g.tibia_length = num("tibia_length", g.tibia_length);
    auto lim = [&](const std::string & name, Limits def) {
        auto v = vec(name, {def.min, def.max});
        return v.size() == 2 ? Limits{v[0], v[1]} : def;
      };
    g.coxa_limits = lim("coxa_limits", g.coxa_limits);
    g.femur_limits = lim("femur_limits", g.femur_limits);
    g.tibia_limits = lim("tibia_limits", g.tibia_limits);
    g.body_height_range = lim("body_height_range", g.body_height_range);
    g.stance_radius = num("stance_radius", g.stance_radius);
    g.body_height = num("body_height", g.body_height);
    g.step_height = num("step_height", g.step_height);
    g.step_height_max = num("step_height_max", g.step_height_max);
    // Declared so the shared geometry.yaml loads without "unused" noise.
    for (const char * extra : {"tray_thickness", "deck_z", "head_tilt_height", "depth_camera_pitch",
        "mass_base", "mass_coxa", "mass_femur", "mass_tibia", "mass_head_pan", "mass_head_tilt"})
    {
      num(extra, 0.0);
    }
    for (const char * extra : {"head_pan_xyz", "head_pan_limits", "head_tilt_limits",
        "payload_lrf_xyz", "payload_lowlight_xyz", "payload_thermal_xyz",
        "payload_illuminator_xyz", "lidar_xyz", "depth_camera_xyz", "gnss_xyz"})
    {
      vec(extra, {0.0});
    }
    declare_parameter("geometry.servo_ids", std::vector<int64_t>{});
    return g;
  }

  void onTwist(const geometry_msgs::msg::Twist & t)
  {
    target_cmd_ = {t.linear.x, t.linear.y, t.angular.z};
    last_cmd_time_ = now();
  }

  void onJointStates(const sensor_msgs::msg::JointState & m)
  {
    int found = 0;
    for (size_t i = 0; i < m.name.size(); ++i) {
      auto it = joint_index_.find(m.name[i]);
      if (it == joint_index_.end()) {continue;}
      if (i < m.position.size()) {
        measured_[it->second] = m.position[i];
        ++found;
      }
      if (i < m.effort.size()) {effort_[it->second] = m.effort[i];}
    }
    if (found == kNumLegJoints) {
      have_joints_ = true;
    }
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const auto & q = m.orientation;
    if (q.w == 0.0 && q.x == 0.0 && q.y == 0.0 && q.z == 0.0) {return;}
    imu_roll_ = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    imu_pitch_ = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    have_imu_ = true;
  }

  void onTerrain(const hexabot_interfaces::msg::TerrainInfo & m)
  {
    if (!auto_terrain_) {return;}
    if (m.recommended_step_height > 0.0) {
      gait_->setStepHeight(std::max<double>(m.recommended_step_height, geo_.step_height));
    }
    if (m.recommended_body_height > 0.0 && !crouched_) {
      height_target_ = geo_.body_height_range.clamp(m.recommended_body_height);
    }
  }

  void onSetMode(const SetMode::Request & req, SetMode::Response & res)
  {
    res.success = true;
    if (req.mode == "stand") {
      if (mode_ == Mode::kStanding) {
        res.message = "already standing";
      } else if (!have_joints_) {
        res.success = false;
        res.message = "no joint_states yet";
      } else {
        if (mode_ == Mode::kRelaxed) {
          setHardwareActive(true);
        }
        beginStandUp();
        res.message = "standing up";
      }
    } else if (req.mode == "sit") {
      if (mode_ == Mode::kStanding) {
        sit_requested_ = true;
        res.message = "sitting down";
      } else {
        res.success = false;
        res.message = std::string("cannot sit from mode ") + modeName(mode_);
      }
    } else if (req.mode == "crouch") {
      crouched_ = true;
      height_target_ = crouch_height_;
      gait_->setStepHeight(std::min(gait_->stepHeight(), 0.03));
      res.message = "crouched (stealth)";
    } else if (req.mode == "normal") {
      crouched_ = false;
      height_target_ = nominal_height_;
      gait_->setStepHeight(geo_.step_height);
      res.message = "normal height";
    } else if (req.mode == "relax") {
      if (mode_ == Mode::kSitting || mode_ == Mode::kFallen || mode_ == Mode::kUnknown) {
        setHardwareActive(false);
        mode_ = Mode::kRelaxed;
        res.message = "servo torque off";
      } else {
        res.success = false;
        res.message = "sit down first";
      }
    } else {
      res.success = false;
      res.message = "unknown mode (stand | sit | crouch | normal | relax)";
    }
  }

  void setHardwareActive(bool active)
  {
    if (!hw_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "controller_manager hardware state service not available");
      return;
    }
    auto req = std::make_shared<SetHwState::Request>();
    req->name = hw_component_;
    req->target_state.id = active ? lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE :
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    req->target_state.label = active ? "active" : "inactive";
    hw_client_->async_send_request(req);
  }

  std::vector<double> jointsForFeet(const std::array<Vec3, kNumLegs> & feet, double height)
  {
    std::vector<double> q(kNumLegJoints, 0.0);
    BodyPose pose = user_pose_;
    pose.roll += level_roll_;
    pose.pitch += level_pitch_;
    bool all_ok = true;
    for (int i = 0; i < kNumLegs; ++i) {
      const Vec3 p_base = stanceToBase(feet[i], pose, height);
      const IkResult r = inverseKinematics(geo_, i, p_base);
      all_ok &= r.reachable && r.within_limits;
      q[3 * i + 0] = r.q.coxa;
      q[3 * i + 1] = r.q.femur;
      q[3 * i + 2] = r.q.tibia;
    }
    if (!all_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "foot target outside the leg workspace, joints clamped");
    }
    return q;
  }

  std::array<Vec3, kNumLegs> neutralFeet() const
  {
    std::array<Vec3, kNumLegs> f{};
    for (int i = 0; i < kNumLegs; ++i) {f[i] = geo_.neutralFoot(i);}
    return f;
  }

  void beginStandUp()
  {
    gait_->reset();
    body_height_ = sit_height_;
    // Phase 1: gather the legs into the sitting pose (body resting on the ground).
    const auto sit = jointsForFeet(neutralFeet(), sit_height_);
    joint_interp_.start(measured_, sit, transition_time_);
    stand_phase_ = 1;
    mode_ = Mode::kStandingUp;
  }

  void tick()
  {
    const double dt = 1.0 / rate_;
    const auto t_now = now();

    if (mode_ == Mode::kUnknown && have_joints_ && start_mode_ == "stand") {
      beginStandUp();
    }

    // Fall detection.
    if (have_imu_ && (std::abs(imu_roll_) > fall_angle_ || std::abs(imu_pitch_) > fall_angle_)) {
      fall_time_ += dt;
      if (fall_time_ > 0.3 && mode_ != Mode::kFallen && mode_ != Mode::kRelaxed) {
        RCLCPP_ERROR(get_logger(), "fall detected (roll %.2f, pitch %.2f)", imu_roll_, imu_pitch_);
        mode_ = Mode::kFallen;
        gait_->reset();
      }
    } else {
      fall_time_ = 0.0;
    }

    std::optional<std::vector<double>> command;
    switch (mode_) {
      case Mode::kStandingUp:
        if (stand_phase_ == 1) {
          command = joint_interp_.update(dt);
          if (!joint_interp_.active()) {
            stand_phase_ = 2;
            height_interp_.start({sit_height_}, {height_target_}, transition_time_);
          }
        } else {
          body_height_ = height_interp_.update(dt)[0];
          command = jointsForFeet(gait_->feet(), body_height_ + gait_->meanGroundHeight());
          if (!height_interp_.active()) {
            mode_ = Mode::kStanding;
            RCLCPP_INFO(get_logger(), "standing");
          }
        }
        break;
      case Mode::kStanding: {
          Twist2D cmd = target_cmd_;
          if ((t_now - last_cmd_time_).seconds() > cmd_timeout_ || sit_requested_) {
            cmd = {};
          }
          gait_->setCommand(cmd);
          updateFeedback(dt);
          gait_->update(dt);
          body_height_ += std::clamp(height_target_ - body_height_, -height_rate_ * dt,
              height_rate_ * dt);
          command = jointsForFeet(gait_->feet(), body_height_ + gait_->meanGroundHeight());
          if (sit_requested_ && !gait_->active()) {
            sit_requested_ = false;
            height_interp_.start({body_height_}, {sit_height_}, transition_time_);
            mode_ = Mode::kSittingDown;
          }
          break;
        }
      case Mode::kSittingDown:
        body_height_ = height_interp_.update(dt)[0];
        command = jointsForFeet(gait_->feet(), body_height_ + gait_->meanGroundHeight());
        if (!height_interp_.active()) {
          mode_ = Mode::kSitting;
          RCLCPP_INFO(get_logger(), "sitting");
        }
        break;
      default:
        break;
    }

    if (command) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = *command;
      cmd_pub_->publish(msg);
      last_command_ = *command;
    }

    updateOdometry(dt, t_now);
    if (++state_counter_ >= static_cast<int>(rate_ / 10.0)) {
      state_counter_ = 0;
      publishState(t_now);
    }
  }

  void updateFeedback(double dt)
  {
    if (leveling_ && have_imu_) {
      level_roll_ = std::clamp(level_roll_ - leveling_gain_ * imu_roll_ * dt,
          -leveling_limit_, leveling_limit_);
      level_pitch_ = std::clamp(level_pitch_ - leveling_gain_ * imu_pitch_ * dt,
          -leveling_limit_, leveling_limit_);
    }
    if (contact_threshold_ > 0.0) {
      for (int i = 0; i < kNumLegs; ++i) {
        gait_->setFootContact(i, std::abs(effort_[3 * i + 2]) > contact_threshold_);
      }
    }
  }

  void updateOdometry(double dt, const rclcpp::Time & stamp)
  {
    if (mode_ != Mode::kStanding) {
      odom_.reset(odom_.pose());
      publishOdometry(stamp, {}, dt);
      return;
    }
    Pose2D delta{};
    if (odom_from_joints_ && have_joints_) {
      std::array<Vec3, kNumLegs> feet{};
      for (int i = 0; i < kNumLegs; ++i) {
        feet[i] = forwardKinematics(geo_, i,
            {measured_[3 * i], measured_[3 * i + 1], measured_[3 * i + 2]});
      }
      if (odom_.update(feet, gait_->inStance())) {
        delta = odom_.lastDelta();
      }
    } else {
      const auto & c = gait_->command();
      delta = {c.vx * dt, c.vy * dt, c.wz * dt};
      odom_.reset(integratePose(odom_.pose(), delta));
    }
    publishOdometry(stamp, delta, dt);
  }

  void publishOdometry(const rclcpp::Time & stamp, const Pose2D & delta, double dt)
  {
    const Pose2D & p = odom_.pose();
    const double alpha = 0.2;
    vel_.x = (1 - alpha) * vel_.x + alpha * delta.x / dt;
    vel_.y = (1 - alpha) * vel_.y + alpha * delta.y / dt;
    vel_.yaw = (1 - alpha) * vel_.yaw + alpha * delta.yaw / dt;
    if (++odom_counter_ < 2) {return;}   // 50 Hz
    odom_counter_ = 0;

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = footprint_frame_;
    o.pose.pose.position.x = p.x;
    o.pose.pose.position.y = p.y;
    o.pose.pose.position.z = 0.0;
    o.pose.pose.orientation.z = std::sin(p.yaw / 2.0);
    o.pose.pose.orientation.w = std::cos(p.yaw / 2.0);
    o.twist.twist.linear.x = vel_.x;
    o.twist.twist.linear.y = vel_.y;
    o.twist.twist.angular.z = vel_.yaw;
    for (int i : {0, 7, 35}) {o.pose.covariance[i] = 0.01;}
    for (int i : {14, 21, 28}) {o.pose.covariance[i] = 1e3;}
    for (int i : {0, 7, 35}) {o.twist.covariance[i] = 0.004;}
    for (int i : {14, 21, 28}) {o.twist.covariance[i] = 1e3;}
    odom_pub_->publish(o);

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = o.header;
      t.child_frame_id = footprint_frame_;
      t.transform.translation.x = p.x;
      t.transform.translation.y = p.y;
      t.transform.rotation = o.pose.pose.orientation;
      tfs.push_back(t);
    }
    // Footprint (ground under the body) -> base_link: height and body pose.
    geometry_msgs::msg::TransformStamped b;
    b.header.stamp = stamp;
    b.header.frame_id = footprint_frame_;
    b.child_frame_id = base_frame_;
    b.transform.translation.x = user_pose_.x;
    b.transform.translation.y = user_pose_.y;
    b.transform.translation.z = body_height_ + user_pose_.z + gait_->meanGroundHeight();
    const double r = user_pose_.roll + level_roll_, pt = user_pose_.pitch + level_pitch_;
    const double y = user_pose_.yaw;
    const double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(pt / 2),
      sp = std::sin(pt / 2), cy = std::cos(y / 2), sy = std::sin(y / 2);
    b.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    b.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    b.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    b.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    tfs.push_back(b);
    tf_->sendTransform(tfs);
  }

  void publishState(const rclcpp::Time & stamp)
  {
    hexabot_interfaces::msg::LocomotionState s;
    s.header.stamp = stamp;
    s.header.frame_id = base_frame_;
    s.mode = modeName(mode_);
    s.gait = gaitName(gait_->gait());
    s.walking = gait_->active();
    s.crouched = crouched_;
    s.phase = static_cast<float>(gait_->phase());
    const auto st = gait_->inStance();
    for (int i = 0; i < kNumLegs; ++i) {s.stance[i] = st[i];}
    s.body_height = static_cast<float>(body_height_);
    s.step_height = static_cast<float>(gait_->stepHeight());
    s.command.linear.x = gait_->command().vx;
    s.command.linear.y = gait_->command().vy;
    s.command.angular.z = gait_->command().wz;
    s.roll = static_cast<float>(imu_roll_);
    s.pitch = static_cast<float>(imu_pitch_);
    state_pub_->publish(s);
  }

  Geometry geo_;
  std::unique_ptr<GaitGenerator> gait_;
  JointInterpolator joint_interp_;
  JointInterpolator height_interp_;
  LegOdometry odom_;
  Pose2D vel_{};

  double rate_{100.0};
  double nominal_height_{0.1}, crouch_height_{0.065}, sit_height_{0.03};
  double body_height_{0.1}, height_target_{0.1}, height_rate_{0.05};
  double transition_time_{1.5};
  std::string start_mode_;
  double cmd_timeout_{0.5};
  bool publish_tf_{true};
  std::string odom_frame_, footprint_frame_, base_frame_;
  bool odom_from_joints_{true};
  bool leveling_{false};
  double leveling_gain_{0.8}, leveling_limit_{0.2};
  double level_roll_{0.0}, level_pitch_{0.0};
  double fall_angle_{0.9}, fall_time_{0.0};
  bool auto_terrain_{true};
  double contact_threshold_{0.0};
  std::string hw_component_;

  Mode mode_{Mode::kUnknown};
  int stand_phase_{0};
  bool sit_requested_{false};
  bool crouched_{false};
  Twist2D target_cmd_{};
  rclcpp::Time last_cmd_time_;
  BodyPose user_pose_{};

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, int> joint_index_;
  std::vector<double> measured_, effort_, last_command_;
  bool have_joints_{false};
  bool have_imu_{false};
  double imu_roll_{0.0}, imu_pitch_{0.0};
  int state_counter_{0};
  int odom_counter_{0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<hexabot_interfaces::msg::LocomotionState>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
  rclcpp::Subscription<hexabot_interfaces::msg::BodyPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr step_sub_;
  rclcpp::Subscription<hexabot_interfaces::msg::TerrainInfo>::SharedPtr terrain_sub_;
  rclcpp::Service<SetGait>::SharedPtr gait_srv_;
  rclcpp::Service<SetMode>::SharedPtr mode_srv_;
  rclcpp::Client<SetHwState>::SharedPtr hw_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hexabot_locomotion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hexabot_locomotion::LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
