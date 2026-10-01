// Byte transports for the servo bus: a real serial port and a simulated bus.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace hexabot_hardware
{

class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual bool write(const uint8_t * data, size_t n) = 0;
  /// Reads up to max bytes, waiting at most timeout_ms for the first byte.
  virtual size_t read(uint8_t * buf, size_t max, int timeout_ms) = 0;
  virtual void flushInput() = 0;
  virtual std::string describe() const = 0;
};

/// POSIX serial port (8N1, raw).  Baud rates up to 1 Mbit/s.
class SerialTransport : public Transport
{
public:
  SerialTransport(std::string port, int baudrate);
  ~SerialTransport() override;
  bool open() override;
  void close() override;
  bool isOpen() const override {return fd_ >= 0;}
  bool write(const uint8_t * data, size_t n) override;
  size_t read(uint8_t * buf, size_t max, int timeout_ms) override;
  void flushInput() override;
  std::string describe() const override;
  const std::string & lastError() const {return error_;}

private:
  std::string port_;
  int baudrate_;
  int fd_{-1};
  std::string error_;
};

/// In-memory STS3215 bus for tests and hardware-free runs.  Servos follow
/// their goal position at a realistic speed when torque is enabled.
class SimulatedBus : public Transport
{
public:
  explicit SimulatedBus(const std::vector<uint8_t> & ids, double max_speed_steps = 3400.0);
  bool open() override {open_ = true; return true;}
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}
  bool write(const uint8_t * data, size_t n) override;
  size_t read(uint8_t * buf, size_t max, int timeout_ms) override;
  void flushInput() override;
  std::string describe() const override {return "simulated STS bus";}

  /// Test hooks.
  int position(uint8_t id) const;
  bool torque(uint8_t id) const;
  void setLoad(uint8_t id, int load_permille);
  void dropServo(uint8_t id);
  size_t packetsReceived() const {return packets_;}

private:
  struct Servo
  {
    uint8_t mem[256]{};
    double pos{2048.0};
  };
  void advance();
  void respond(uint8_t id, const std::vector<uint8_t> & params, uint8_t error = 0);
  void handle(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params);

  mutable std::mutex mutex_;
  std::map<uint8_t, Servo> servos_;
  std::deque<uint8_t> rx_;
  std::vector<uint8_t> pending_;
  double max_speed_;
  std::chrono::steady_clock::time_point last_;
  bool open_{false};
  size_t packets_{0};
};

}  // namespace hexabot_hardware
