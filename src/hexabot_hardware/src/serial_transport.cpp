#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

#ifdef __linux__
#include <linux/serial.h>
#endif

#include "hexabot_hardware/transport.hpp"

namespace hexabot_hardware
{

namespace
{
speed_t toSpeed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 1000000: return B1000000;
    default: return B0;
  }
}
}  // namespace

SerialTransport::SerialTransport(std::string port, int baudrate)
: port_(std::move(port)), baudrate_(baudrate) {}

SerialTransport::~SerialTransport() {close();}

bool SerialTransport::open()
{
  close();
  const speed_t speed = toSpeed(baudrate_);
  if (speed == B0) {
    error_ = "unsupported baud rate " + std::to_string(baudrate_);
    return false;
  }
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_ = port_ + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
#ifdef __linux__
  // Ask USB serial drivers (FTDI, CH34x) for low latency; ignore failures.
  serial_struct ss{};
  if (ioctl(fd_, TIOCGSERIAL, &ss) == 0) {
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &ss);
  }
#endif
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialTransport::write(const uint8_t * data, size_t n)
{
  size_t done = 0;
  while (done < n) {
    const ssize_t w = ::write(fd_, data + done, n - done);
    if (w < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        pollfd p{fd_, POLLOUT, 0};
        poll(&p, 1, 5);
        continue;
      }
      error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    done += static_cast<size_t>(w);
  }
  return true;
}

size_t SerialTransport::read(uint8_t * buf, size_t max, int timeout_ms)
{
  pollfd p{fd_, POLLIN, 0};
  const int r = poll(&p, 1, timeout_ms);
  if (r <= 0 || !(p.revents & POLLIN)) {
    return 0;
  }
  const ssize_t n = ::read(fd_, buf, max);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

void SerialTransport::flushInput()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

std::string SerialTransport::describe() const
{
  return port_ + " @ " + std::to_string(baudrate_);
}

}  // namespace hexabot_hardware
