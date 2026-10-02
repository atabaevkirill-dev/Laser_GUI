// Feetech STS (SMS_STS) serial bus protocol, as used by the STS3215.
//
//   instruction: FF FF ID LEN INSTR P1..Pn CHK    LEN = n + 2
//   status:      FF FF ID LEN ERR   P1..Pn CHK
//   CHK = ~(ID + LEN + INSTR/ERR + sum(P)) & 0xFF
//
// Multi-byte values are little-endian.  Present speed has its sign in bit 15,
// present load in bit 10 (sign-magnitude, not two's complement).
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace hexabot_hardware::sts
{

constexpr uint8_t kBroadcastId = 0xFE;

enum Instruction : uint8_t
{
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kRegWrite = 0x04,
  kAction = 0x05,
  kSyncRead = 0x82,
  kSyncWrite = 0x83,
};

// Memory table (SMS_STS series).
enum Address : uint8_t
{
  kId = 5,
  kBaudRate = 6,
  kMinAngleLimit = 9,
  kMaxAngleLimit = 11,
  kOffset = 31,
  kMode = 33,
  kTorqueEnable = 40,
  kAcceleration = 41,
  kGoalPosition = 42,
  kGoalTime = 44,
  kGoalSpeed = 46,
  kTorqueLimit = 48,
  kLock = 55,
  kPresentPosition = 56,
  kPresentSpeed = 58,
  kPresentLoad = 60,
  kPresentVoltage = 62,
  kPresentTemperature = 63,
  kMoving = 66,
  kPresentCurrent = 69,
};

constexpr int kStepsPerRev = 4096;
constexpr int kCenter = 2048;

using Bytes = std::vector<uint8_t>;

uint8_t checksum(uint8_t id, uint8_t len, uint8_t instr, const uint8_t * params, size_t n);

Bytes makePacket(uint8_t id, uint8_t instruction, const Bytes & params = {});
Bytes makePing(uint8_t id);
Bytes makeRead(uint8_t id, uint8_t address, uint8_t length);
Bytes makeWrite(uint8_t id, uint8_t address, const Bytes & data);
/// One SYNC WRITE frame: the same address range for many servos.
Bytes makeSyncWrite(
  uint8_t address, uint8_t length, const std::vector<std::pair<uint8_t, Bytes>> & data);
Bytes makeSyncRead(uint8_t address, uint8_t length, const std::vector<uint8_t> & ids);

inline Bytes u16(int v)
{
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF)};
}
inline int readU16(const uint8_t * p) {return p[0] | (p[1] << 8);}
/// Sign-magnitude decoding (sign in `sign_bit`).
inline int decodeSigned(int raw, int sign_bit)
{
  const int mag = raw & ((1 << sign_bit) - 1);
  return (raw & (1 << sign_bit)) ? -mag : mag;
}
inline int encodeSigned(int v, int sign_bit)
{
  return v < 0 ? ((-v) | (1 << sign_bit)) : v;
}

struct StatusPacket
{
  uint8_t id{0};
  uint8_t error{0};
  Bytes params;
};

/// Incremental parser for status packets; resynchronises on garbage.
class StatusParser
{
public:
  void feed(const uint8_t * data, size_t n);
  void feed(const Bytes & b) {feed(b.data(), b.size());}
  /// Next complete packet with a valid checksum, if any.
  std::optional<StatusPacket> next();
  size_t checksumErrors() const {return checksum_errors_;}
  void clear() {buf_.clear();}

private:
  std::deque<uint8_t> buf_;
  size_t checksum_errors_{0};
};

/// Joint angle <-> servo steps.  offset is the joint angle at step 2048.
int radiansToSteps(double rad, int direction, double offset);
double stepsToRadians(int steps, int direction, double offset);
/// Steps/s -> rad/s for present speed.
double stepsPerSecondToRadians(int steps_per_s, int direction);

}  // namespace hexabot_hardware::sts
