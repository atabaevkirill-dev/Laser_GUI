// ros2_control hardware interface for a bus of Feetech STS3215 servos.
//
// Hardware parameters (URDF <hardware>):
//   port, baudrate               serial bus (default /dev/ttyUSB0 @ 1000000)
//   simulated_bus                true: in-memory bus, no hardware needed
//   acceleration                 0..254, 0 = maximum
//   read_timeout_ms              per SYNC READ
//   max_consecutive_errors       failed reads before the component errors out
//   allow_missing                start even if some servos do not answer
// Joint parameters: id, direction (+1/-1), offset (joint angle at step 2048).
// State interfaces: position, velocity, effort (N*m, from the load reading),
// optional temperature (deg C) and voltage (V).  Command: position.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hexabot_hardware/sts_bus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hexabot_hardware
{

class Sts3215System : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#pragma GCC diagnostic pop

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Stall torque used to scale the load reading into N*m (12 V version).
  static constexpr double kStallTorque = 2.94;

private:
  struct Joint
  {
    std::string name;
    uint8_t id{0};
    int direction{1};
    double offset{0.0};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double temperature{0.0};
    double voltage{0.0};
    double command{std::numeric_limits<double>::quiet_NaN()};
    bool has_temperature{false};
    bool has_voltage{false};
  };

  bool readAll(int timeout_ms);
  std::vector<uint8_t> ids() const;

  std::vector<Joint> joints_;
  std::unique_ptr<StsBus> bus_;
  std::string port_{"/dev/ttyUSB0"};
  int baudrate_{1000000};
  bool simulated_{false};
  int acceleration_{0};
  int read_timeout_ms_{6};
  int max_errors_{50};
  bool allow_missing_{false};
  int consecutive_errors_{0};
  rclcpp::Logger logger_{rclcpp::get_logger("Sts3215System")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace hexabot_hardware
