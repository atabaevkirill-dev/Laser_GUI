// Request/response helpers on top of a Transport.
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "hexabot_hardware/sts_protocol.hpp"
#include "hexabot_hardware/transport.hpp"

namespace hexabot_hardware
{

struct ServoState
{
  int position{0};      ///< steps 0..4095
  int speed{0};         ///< steps/s, signed
  int load{0};          ///< 0.1 % of stall torque, signed
  double voltage{0.0};  ///< V
  int temperature{0};   ///< deg C
};

class StsBus
{
public:
  explicit StsBus(std::unique_ptr<Transport> transport);
  bool open();
  void close();
  bool isOpen() const;
  Transport & transport() {return *transport_;}

  bool ping(uint8_t id, int timeout_ms = 10);
  /// Unicast write; waits for the status packet unless id is broadcast.
  bool write(uint8_t id, uint8_t address, const sts::Bytes & data, int timeout_ms = 10);
  std::optional<sts::Bytes> read(uint8_t id, uint8_t address, uint8_t length, int timeout_ms = 10);
  bool syncWrite(uint8_t address, uint8_t length,
    const std::vector<std::pair<uint8_t, sts::Bytes>> & data);
  /// Responses by id; servos that did not answer within the timeout are absent.
  std::map<uint8_t, sts::Bytes> syncRead(uint8_t address, uint8_t length,
    const std::vector<uint8_t> & ids, int timeout_ms);

  /// Position, speed, load, voltage and temperature of many servos at once.
  std::map<uint8_t, ServoState> readStates(const std::vector<uint8_t> & ids, int timeout_ms);
  bool writeGoalPositions(const std::vector<std::pair<uint8_t, int>> & goals);
  bool setTorque(const std::vector<uint8_t> & ids, bool enable);
  bool setAcceleration(const std::vector<uint8_t> & ids, uint8_t acceleration);

  size_t checksumErrors() const {return parser_.checksumErrors();}

private:
  std::optional<sts::StatusPacket> waitFor(uint8_t id, int timeout_ms);

  std::unique_ptr<Transport> transport_;
  sts::StatusParser parser_;
};

}  // namespace hexabot_hardware
