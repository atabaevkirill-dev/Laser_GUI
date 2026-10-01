#include "hexabot_hardware/sts3215_system.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hexabot_hardware
{

using hardware_interface::return_type;

namespace
{
template<typename T>
T param(const std::unordered_map<std::string, std::string> & p, const std::string & key, T def)
{
  auto it = p.find(key);
  if (it == p.end() || it->second.empty()) {
    return def;
  }
  if constexpr (std::is_same_v<T, bool>) {
    return it->second == "true" || it->second == "True" || it->second == "1";
  } else if constexpr (std::is_same_v<T, int>) {
    return std::stoi(it->second);
  } else if constexpr (std::is_same_v<T, double>) {
    return std::stod(it->second);
  } else {
    return it->second;
  }
}
}  // namespace

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

Sts3215System::CallbackReturn Sts3215System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  try {
    port_ = param<std::string>(hw, "port", port_);
    baudrate_ = param<int>(hw, "baudrate", baudrate_);
    simulated_ = param<bool>(hw, "simulated_bus", false);
    acceleration_ = param<int>(hw, "acceleration", 0);
    read_timeout_ms_ = param<int>(hw, "read_timeout_ms", read_timeout_ms_);
    max_errors_ = param<int>(hw, "max_consecutive_errors", max_errors_);
    allow_missing_ = param<bool>(hw, "allow_missing", false);

    for (const auto & j : info_.joints) {
      Joint joint;
      joint.name = j.name;
      const int id = param<int>(j.parameters, "id", -1);
      if (id < 0 || id > 253) {
        RCLCPP_ERROR(logger_, "joint %s: missing or invalid servo id", j.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint.id = static_cast<uint8_t>(id);
      joint.direction = param<int>(j.parameters, "direction", 1) < 0 ? -1 : 1;
      joint.offset = param<double>(j.parameters, "offset", 0.0);
      if (j.command_interfaces.size() != 1 ||
        j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(logger_, "joint %s: exactly one 'position' command interface expected",
          j.name.c_str());
        return CallbackReturn::ERROR;
      }
      for (const auto & s : j.state_interfaces) {
        joint.has_temperature |= s.name == "temperature";
        joint.has_voltage |= s.name == "voltage";
      }
      joints_.push_back(joint);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "%zu STS3215 joints on %s", joints_.size(),
    simulated_ ? "a simulated bus" : (port_ + " @ " + std::to_string(baudrate_)).c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Sts3215System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity);
    out.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.effort);
    if (j.has_temperature) {out.emplace_back(j.name, "temperature", &j.temperature);}
    if (j.has_voltage) {out.emplace_back(j.name, "voltage", &j.voltage);}
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> Sts3215System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.command);
  }
  return out;
}

#pragma GCC diagnostic pop

std::vector<uint8_t> Sts3215System::ids() const
{
  std::vector<uint8_t> v;
  for (const auto & j : joints_) {v.push_back(j.id);}
  return v;
}

Sts3215System::CallbackReturn Sts3215System::on_configure(const rclcpp_lifecycle::State &)
{
  std::unique_ptr<Transport> t;
  if (simulated_) {
    t = std::make_unique<SimulatedBus>(ids());
  } else {
    t = std::make_unique<SerialTransport>(port_, baudrate_);
  }
  bus_ = std::make_unique<StsBus>(std::move(t));
  if (!bus_->open()) {
    auto * serial = dynamic_cast<SerialTransport *>(&bus_->transport());
    RCLCPP_ERROR(logger_, "cannot open servo bus: %s",
      serial ? serial->lastError().c_str() : "unknown error");
    return CallbackReturn::ERROR;
  }
  std::vector<std::string> missing;
  for (const auto & j : joints_) {
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      ok = bus_->ping(j.id, 20);
    }
    if (!ok) {missing.push_back(j.name + " (id " + std::to_string(j.id) + ")");}
  }
  if (!missing.empty()) {
    std::string list;
    for (const auto & m : missing) {list += " " + m;}
    RCLCPP_ERROR(logger_, "servos not answering:%s", list.c_str());
    if (!allow_missing_) {
      return CallbackReturn::ERROR;
    }
  }
  if (!readAll(50)) {
    RCLCPP_WARN(logger_, "initial state read incomplete");
  }
  for (auto & j : joints_) {
    j.command = std::numeric_limits<double>::quiet_NaN();
  }
  RCLCPP_INFO(logger_, "servo bus configured (%s)", bus_->transport().describe().c_str());
  return CallbackReturn::SUCCESS;
}

Sts3215System::CallbackReturn Sts3215System::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (bus_) {
    bus_->close();
    bus_.reset();
  }
  return CallbackReturn::SUCCESS;
}

Sts3215System::CallbackReturn Sts3215System::on_activate(const rclcpp_lifecycle::State &)
{
  if (!bus_) {
    return CallbackReturn::ERROR;
  }
  readAll(50);
  // Hold the current pose: goal = present position before enabling torque.
  std::vector<std::pair<uint8_t, int>> goals;
  for (auto & j : joints_) {
    j.command = j.position;
    goals.emplace_back(j.id, sts::radiansToSteps(j.position, j.direction, j.offset));
  }
  bus_->writeGoalPositions(goals);
  bus_->setAcceleration(ids(), static_cast<uint8_t>(acceleration_));
  if (!bus_->setTorque(ids(), true)) {
    RCLCPP_ERROR(logger_, "could not enable torque");
    return CallbackReturn::ERROR;
  }
  consecutive_errors_ = 0;
  RCLCPP_INFO(logger_, "torque enabled on %zu servos", joints_.size());
  return CallbackReturn::SUCCESS;
}

Sts3215System::CallbackReturn Sts3215System::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (bus_) {
    bus_->setTorque(ids(), false);
    RCLCPP_INFO(logger_, "torque disabled (robot is limp)");
  }
  return CallbackReturn::SUCCESS;
}

Sts3215System::CallbackReturn Sts3215System::on_shutdown(const rclcpp_lifecycle::State & s)
{
  on_deactivate(s);
  return on_cleanup(s);
}

bool Sts3215System::readAll(int timeout_ms)
{
  const auto states = bus_->readStates(ids(), timeout_ms);
  for (auto & j : joints_) {
    auto it = states.find(j.id);
    if (it == states.end()) {continue;}
    const ServoState & s = it->second;
    j.position = sts::stepsToRadians(s.position, j.direction, j.offset);
    j.velocity = sts::stepsPerSecondToRadians(s.speed, j.direction);
    j.effort = j.direction * s.load * 0.001 * kStallTorque;
    j.temperature = s.temperature;
    j.voltage = s.voltage;
  }
  return states.size() == joints_.size();
}

return_type Sts3215System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {
    return return_type::ERROR;
  }
  if (readAll(read_timeout_ms_)) {
    consecutive_errors_ = 0;
  } else if (++consecutive_errors_ > max_errors_) {
    RCLCPP_ERROR(logger_, "%d consecutive incomplete bus reads, giving up", consecutive_errors_);
    return return_type::ERROR;
  } else {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000, "incomplete servo read (%d in a row)",
      consecutive_errors_);
  }
  return return_type::OK;
}

return_type Sts3215System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {
    return return_type::ERROR;
  }
  std::vector<std::pair<uint8_t, int>> goals;
  goals.reserve(joints_.size());
  for (const auto & j : joints_) {
    if (std::isfinite(j.command)) {
      goals.emplace_back(j.id, sts::radiansToSteps(j.command, j.direction, j.offset));
    }
  }
  return bus_->writeGoalPositions(goals) ? return_type::OK : return_type::ERROR;
}

}  // namespace hexabot_hardware

PLUGINLIB_EXPORT_CLASS(hexabot_hardware::Sts3215System, hardware_interface::SystemInterface)
