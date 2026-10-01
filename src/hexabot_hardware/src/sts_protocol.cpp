#include "hexabot_hardware/sts_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace hexabot_hardware::sts
{

uint8_t checksum(uint8_t id, uint8_t len, uint8_t instr, const uint8_t * params, size_t n)
{
  unsigned sum = id + len + instr;
  for (size_t i = 0; i < n; ++i) {
    sum += params[i];
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

Bytes makePacket(uint8_t id, uint8_t instruction, const Bytes & params)
{
  const auto len = static_cast<uint8_t>(params.size() + 2);
  Bytes p{0xFF, 0xFF, id, len, instruction};
  p.insert(p.end(), params.begin(), params.end());
  p.push_back(checksum(id, len, instruction, params.data(), params.size()));
  return p;
}

Bytes makePing(uint8_t id) {return makePacket(id, kPing);}

Bytes makeRead(uint8_t id, uint8_t address, uint8_t length)
{
  return makePacket(id, kRead, {address, length});
}

Bytes makeWrite(uint8_t id, uint8_t address, const Bytes & data)
{
  Bytes params{address};
  params.insert(params.end(), data.begin(), data.end());
  return makePacket(id, kWrite, params);
}

Bytes makeSyncWrite(
  uint8_t address, uint8_t length, const std::vector<std::pair<uint8_t, Bytes>> & data)
{
  Bytes params{address, length};
  for (const auto & [id, bytes] : data) {
    params.push_back(id);
    for (uint8_t k = 0; k < length; ++k) {
      params.push_back(k < bytes.size() ? bytes[k] : 0);
    }
  }
  return makePacket(kBroadcastId, kSyncWrite, params);
}

Bytes makeSyncRead(uint8_t address, uint8_t length, const std::vector<uint8_t> & ids)
{
  Bytes params{address, length};
  params.insert(params.end(), ids.begin(), ids.end());
  return makePacket(kBroadcastId, kSyncRead, params);
}

void StatusParser::feed(const uint8_t * data, size_t n)
{
  buf_.insert(buf_.end(), data, data + n);
}

std::optional<StatusPacket> StatusParser::next()
{
  while (buf_.size() >= 6) {
    if (buf_[0] != 0xFF || buf_[1] != 0xFF) {
      buf_.pop_front();
      continue;
    }
    const uint8_t id = buf_[2];
    const uint8_t len = buf_[3];
    if (id == 0xFF || len < 2) {   // "FF FF FF ..." -> header starts one byte later
      buf_.pop_front();
      continue;
    }
    const size_t total = 4 + static_cast<size_t>(len);
    if (buf_.size() < total) {
      return std::nullopt;
    }
    const uint8_t error = buf_[4];
    Bytes params(buf_.begin() + 5, buf_.begin() + static_cast<long>(total) - 1);
    const uint8_t chk = buf_[total - 1];
    if (chk != checksum(id, len, error, params.data(), params.size())) {
      ++checksum_errors_;
      buf_.pop_front();
      continue;
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(total));
    return StatusPacket{id, error, std::move(params)};
  }
  return std::nullopt;
}

int radiansToSteps(double rad, int direction, double offset)
{
  const double steps = kCenter + direction * (rad - offset) * kStepsPerRev / (2.0 * M_PI);
  return std::clamp(static_cast<int>(std::lround(steps)), 0, kStepsPerRev - 1);
}

double stepsToRadians(int steps, int direction, double offset)
{
  return offset + direction * (steps - kCenter) * 2.0 * M_PI / kStepsPerRev;
}

double stepsPerSecondToRadians(int steps_per_s, int direction)
{
  return direction * steps_per_s * 2.0 * M_PI / kStepsPerRev;
}

}  // namespace hexabot_hardware::sts
