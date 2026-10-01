#include <algorithm>
#include <cmath>
#include <cstring>

#include "hexabot_hardware/sts_protocol.hpp"
#include "hexabot_hardware/transport.hpp"

namespace hexabot_hardware
{

using namespace sts;

SimulatedBus::SimulatedBus(const std::vector<uint8_t> & ids, double max_speed_steps)
: max_speed_(max_speed_steps), last_(std::chrono::steady_clock::now())
{
  for (uint8_t id : ids) {
    Servo s;
    s.mem[kId] = id;
    s.mem[kPresentVoltage] = 121;   // 12.1 V
    s.mem[kPresentTemperature] = 34;
    s.mem[kGoalPosition] = kCenter & 0xFF;
    s.mem[kGoalPosition + 1] = kCenter >> 8;
    servos_[id] = s;
  }
}

void SimulatedBus::advance()
{
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_).count();
  last_ = now;
  for (auto & [id, s] : servos_) {
    (void)id;
    const double goal = readU16(&s.mem[kGoalPosition]);
    double speed = 0.0;
    if (s.mem[kTorqueEnable]) {
      const double step = std::clamp(goal - s.pos, -max_speed_ * dt, max_speed_ * dt);
      s.pos += step;
      speed = dt > 0 ? step / dt : 0.0;
    }
    const int p = static_cast<int>(std::lround(s.pos));
    s.mem[kPresentPosition] = p & 0xFF;
    s.mem[kPresentPosition + 1] = (p >> 8) & 0xFF;
    const int sp = encodeSigned(static_cast<int>(std::lround(speed)), 15);
    s.mem[kPresentSpeed] = sp & 0xFF;
    s.mem[kPresentSpeed + 1] = (sp >> 8) & 0xFF;
  }
}

void SimulatedBus::respond(uint8_t id, const std::vector<uint8_t> & params, uint8_t error)
{
  const auto len = static_cast<uint8_t>(params.size() + 2);
  rx_.insert(rx_.end(), {0xFF, 0xFF, id, len, error});
  rx_.insert(rx_.end(), params.begin(), params.end());
  rx_.push_back(checksum(id, len, error, params.data(), params.size()));
}

void SimulatedBus::handle(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params)
{
  ++packets_;
  advance();
  auto it = servos_.find(id);
  switch (instr) {
    case kPing:
      if (it != servos_.end()) {respond(id, {});}
      break;
    case kRead:
      if (it != servos_.end() && params.size() >= 2) {
        const size_t a = params[0], n = std::min<size_t>(params[1], 256 - a);
        respond(id, std::vector<uint8_t>(it->second.mem + a, it->second.mem + a + n));
      }
      break;
    case kWrite:
      if (params.empty()) {break;}
      for (auto & [sid, s] : servos_) {
        if (id != kBroadcastId && sid != id) {continue;}
        std::memcpy(s.mem + params[0], params.data() + 1, params.size() - 1);
      }
      if (it != servos_.end()) {respond(id, {});}
      break;
    case kSyncWrite: {
        if (params.size() < 2) {break;}
        const uint8_t a = params[0], n = params[1];
        for (size_t k = 2; k + n < params.size(); k += 1 + n) {
          auto s = servos_.find(params[k]);
          if (s != servos_.end()) {std::memcpy(s->second.mem + a, &params[k + 1], n);}
        }
        break;
      }
    case kSyncRead: {
        if (params.size() < 2) {break;}
        const size_t a = params[0], n = std::min<size_t>(params[1], 256 - a);
        for (size_t k = 2; k < params.size(); ++k) {
          auto s = servos_.find(params[k]);
          if (s != servos_.end()) {
            respond(params[k], std::vector<uint8_t>(s->second.mem + a, s->second.mem + a + n));
          }
        }
        break;
      }
    default:
      break;
  }
}

bool SimulatedBus::write(const uint8_t * data, size_t n)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.insert(pending_.end(), data, data + n);
  // Parse complete instruction packets.
  while (pending_.size() >= 6) {
    if (pending_[0] != 0xFF || pending_[1] != 0xFF) {
      pending_.erase(pending_.begin());
      continue;
    }
    const uint8_t len = pending_[3];
    const size_t total = 4 + static_cast<size_t>(len);
    if (pending_.size() < total) {break;}
    const uint8_t id = pending_[2], instr = pending_[4];
    std::vector<uint8_t> params(pending_.begin() + 5, pending_.begin() + static_cast<long>(total) - 1);
    if (pending_[total - 1] == checksum(id, len, instr, params.data(), params.size())) {
      handle(id, instr, params);
    }
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<long>(total));
  }
  return open_;
}

size_t SimulatedBus::read(uint8_t * buf, size_t max, int /*timeout_ms*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t n = 0;
  while (n < max && !rx_.empty()) {
    buf[n++] = rx_.front();
    rx_.pop_front();
  }
  return n;
}

void SimulatedBus::flushInput()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rx_.clear();
}

int SimulatedBus::position(uint8_t id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = servos_.find(id);
  return it == servos_.end() ? -1 : static_cast<int>(std::lround(it->second.pos));
}

bool SimulatedBus::torque(uint8_t id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = servos_.find(id);
  return it != servos_.end() && it->second.mem[kTorqueEnable];
}

void SimulatedBus::setLoad(uint8_t id, int load_permille)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = servos_.find(id);
  if (it == servos_.end()) {return;}
  const int v = encodeSigned(load_permille, 10);
  it->second.mem[kPresentLoad] = v & 0xFF;
  it->second.mem[kPresentLoad + 1] = (v >> 8) & 0xFF;
}

void SimulatedBus::dropServo(uint8_t id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  servos_.erase(id);
}

}  // namespace hexabot_hardware
