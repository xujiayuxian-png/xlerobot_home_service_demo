#include "xlerobot_lidar_driver/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <thread>

#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" int tcdrain(int fd);

namespace xlerobot_lidar_driver
{

SerialPort::~SerialPort()
{
  closePort();
}

bool SerialPort::openPort(const std::string & port, int baudrate)
{
  closePort();
  constexpr int kMaxOpenAttempts = 5;
  for (int attempt = 1; attempt <= kMaxOpenAttempts; ++attempt) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ != -1) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100 * attempt));
  }
  if (fd_ == -1) {
    return false;
  }

  struct termios2 tio;
  if (ioctl(fd_, TCGETS2, &tio) != 0) {
    closePort();
    return false;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8 | CREAD | CLOCAL;

  // The lidar stream is binary. Any tty byte translation corrupts packets and
  // shows up as checksum failures, especially CR (0x0D) -> NL (0x0A).
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
  tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
    IXON | IXOFF | IXANY);
  tio.c_oflag &= ~OPOST;

  if (ioctl(fd_, TCSETS2, &tio) != 0) {
    closePort();
    return false;
  }

  ioctl(fd_, TCFLSH, TCIOFLUSH);
  return true;
}

void SerialPort::closePort()
{
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ != -1;
}

int SerialPort::readBytes(uint8_t * buffer, size_t size)
{
  if (fd_ == -1) {
    errno = EBADF;
    return -1;
  }
  return ::read(fd_, buffer, size);
}

bool SerialPort::writeBytes(const std::vector<uint8_t> & bytes)
{
  if (fd_ == -1) {
    return false;
  }

  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t written = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (written > 0) {
      offset += static_cast<size_t>(written);
      continue;
    }
    if (written == -1 && errno == EINTR) {
      continue;
    }
    if (written == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    return false;
  }
  return true;
}

void SerialPort::sendStopCommand()
{
  if (fd_ == -1) {
    return;
  }
  const std::vector<uint8_t> stop_cmd = {0xA5, 0x65};
  writeBytes(stop_cmd);
  tcdrain(fd_);
}

}  // namespace xlerobot_lidar_driver
