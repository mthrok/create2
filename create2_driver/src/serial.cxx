#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <stdexcept>

#include "create2_driver/serial.hpp"

Serial::Serial()
  : fd_()
  , oldtio_()
{};

void Serial::init(const int baudrate, const char* device) {
  struct termios toptions;
  fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd_ == -1)  {
    std::runtime_error("Failed to open device.");
  }
  if (tcgetattr(fd_, &toptions) < 0) {
    std::runtime_error("Failed to get term attributes.");
  }

  cfsetispeed(&toptions, baudrate);
  cfsetospeed(&toptions, baudrate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  toptions.c_cflag &= ~CRTSCTS;
  toptions.c_cflag |= CREAD | CLOCAL;
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  toptions.c_oflag &= ~OPOST;
  toptions.c_cc[VMIN]  = 26;
  toptions.c_cc[VTIME] = 1;
  if( tcsetattr(fd_, TCSANOW, &toptions) < 0) {
    std::runtime_error("Failed to set term term attributes.");
  }
}

Serial::~Serial() {
  ::close(fd_);
}


int Serial::read(uint8_t* pBuffer, int len) {
  return ::read(fd_, pBuffer, len);
}

int Serial::write(const uint8_t* pData, int len) {
  return ::write(fd_, pData, len);
}

void Serial::setRts(int sw) {
  int status;
  ioctl(fd_, TIOCMGET, &status); // set the serial port status

  if (sw)
    status &= ~TIOCM_RTS;
  else
    status |= TIOCM_RTS;

  ioctl(fd_, TIOCMSET, &status); // set the serial port status
}
