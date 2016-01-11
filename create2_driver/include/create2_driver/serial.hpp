#ifndef CREATE2_DRIVER_SERIAL_HPP
#define CREATE2_DRIVER_SERIAL_HPP

#include <stdint.h>
#include <termios.h>

class Serial {

private:
  int fd_;
public:
  Serial();
  ~Serial();

  void open(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");
  void close();
  int read(uint8_t* pBuffer, int len);
  int write(const uint8_t* pData, int len);
  //void setRts(int);
};

#endif // CREATE2_DRIVER_SERIAL_HPP
