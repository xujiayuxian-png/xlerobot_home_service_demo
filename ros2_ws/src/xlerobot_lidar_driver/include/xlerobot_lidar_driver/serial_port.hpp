#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace xlerobot_lidar_driver
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool openPort(const std::string & port, int baudrate);
  void closePort();
  bool isOpen() const;

  int readBytes(uint8_t * buffer, size_t size);
  bool writeBytes(const std::vector<uint8_t> & bytes);
  void sendStopCommand();

private:
  int fd_ = -1;
};

}  // namespace xlerobot_lidar_driver
