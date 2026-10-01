#include "hexabot_hardware/sts_bus.hpp"

#include <chrono>

namespace hexabot_hardware
{

using namespace sts;
using Clock = std::chrono::steady_clock;

StsBus::StsBus(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {}

bool StsBus::open() {return transport_->open();}
void StsBus::close() {transport_->close();}
bool StsBus::isOpen() const {return transport_->isOpen();}

std::optional<StatusPacket> StsBus::waitFor(uint8_t id, int timeout_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t buf[256];
  while (true) {
    while (auto p = parser_.next()) {
      if (p->id == id) {
        return p;
      }
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (left.count() < 0) {
      return std::nullopt;
    }
    const size_t n = transport_->read(buf, sizeof(buf), static_cast<int>(left.count()));
    if (n > 0) {
      parser_.feed(buf, n);
    } else if (left.count() == 0) {
      return std::nullopt;
    }
  }
}

bool StsBus::ping(uint8_t id, int timeout_ms)
{
  transport_->flushInput();
  parser_.clear();
  const Bytes p = makePing(id);
  return transport_->write(p.data(), p.size()) && waitFor(id, timeout_ms).has_value();
}

bool StsBus::write(uint8_t id, uint8_t address, const Bytes & data, int timeout_ms)
{
  transport_->flushInput();
  parser_.clear();
  const Bytes p = makeWrite(id, address, data);
  if (!transport_->write(p.data(), p.size())) {
    return false;
  }
  if (id == kBroadcastId) {
    return true;
  }
  auto s = waitFor(id, timeout_ms);
  return s && s->error == 0;
}

std::optional<Bytes> StsBus::read(uint8_t id, uint8_t address, uint8_t length, int timeout_ms)
{
  transport_->flushInput();
  parser_.clear();
  const Bytes p = makeRead(id, address, length);
  if (!transport_->write(p.data(), p.size())) {
    return std::nullopt;
  }
  auto s = waitFor(id, timeout_ms);
  if (!s || s->params.size() != length) {
    return std::nullopt;
  }
  return s->params;
}

bool StsBus::syncWrite(
  uint8_t address, uint8_t length, const std::vector<std::pair<uint8_t, Bytes>> & data)
{
  if (data.empty()) {
    return true;
  }
  const Bytes p = makeSyncWrite(address, length, data);
  return transport_->write(p.data(), p.size());
}

std::map<uint8_t, Bytes> StsBus::syncRead(
  uint8_t address, uint8_t length, const std::vector<uint8_t> & ids, int timeout_ms)
{
  std::map<uint8_t, Bytes> out;
  transport_->flushInput();
  parser_.clear();
  const Bytes p = makeSyncRead(address, length, ids);
  if (!transport_->write(p.data(), p.size())) {
    return out;
  }
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t buf[512];
  while (out.size() < ids.size()) {
    while (auto s = parser_.next()) {
      if (s->params.size() == length) {
        out[s->id] = std::move(s->params);
      }
    }
    if (out.size() == ids.size()) {
      break;
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (left.count() < 0) {
      break;
    }
    const size_t n = transport_->read(buf, sizeof(buf), static_cast<int>(left.count()));
    if (n > 0) {
      parser_.feed(buf, n);
    } else if (left.count() == 0) {
      break;
    }
  }
  return out;
}

std::map<uint8_t, ServoState> StsBus::readStates(const std::vector<uint8_t> & ids, int timeout_ms)
{
  std::map<uint8_t, ServoState> out;
  for (const auto & [id, b] : syncRead(kPresentPosition, 8, ids, timeout_ms)) {
    ServoState s;
    s.position = readU16(&b[0]);
    s.speed = decodeSigned(readU16(&b[2]), 15);
    s.load = decodeSigned(readU16(&b[4]), 10);
    s.voltage = b[6] * 0.1;
    s.temperature = b[7];
    out[id] = s;
  }
  return out;
}

bool StsBus::writeGoalPositions(const std::vector<std::pair<uint8_t, int>> & goals)
{
  std::vector<std::pair<uint8_t, Bytes>> data;
  data.reserve(goals.size());
  for (const auto & [id, steps] : goals) {
    data.emplace_back(id, u16(steps));
  }
  return syncWrite(kGoalPosition, 2, data);
}

bool StsBus::setTorque(const std::vector<uint8_t> & ids, bool enable)
{
  std::vector<std::pair<uint8_t, Bytes>> data;
  for (uint8_t id : ids) {
    data.emplace_back(id, Bytes{static_cast<uint8_t>(enable ? 1 : 0)});
  }
  return syncWrite(kTorqueEnable, 1, data);
}

bool StsBus::setAcceleration(const std::vector<uint8_t> & ids, uint8_t acceleration)
{
  std::vector<std::pair<uint8_t, Bytes>> data;
  for (uint8_t id : ids) {
    data.emplace_back(id, Bytes{acceleration});
  }
  return syncWrite(kAcceleration, 1, data);
}

}  // namespace hexabot_hardware
