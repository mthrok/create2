#include "create2_driver/create2_driver.hpp"

void sleep_for_sec (float sec) {
  long usec=(long)(sec*1000000);
  usleep(usec);
}

Create2::Create2()
  : comm_()
{};

Create2::~Create2() {};

void Create2::init(const int baudrate, const char* device) {
  comm_.init(baudrate, device);
};

void Create2::sendOpCode(const OPCODE code, const uint32_t size, const uint8_t* pData) {
  uint8_t* pBuffer = new uint8_t[size + 1];
  pBuffer[0] = (uint8_t)code;
  memcpy(pBuffer+1, pData, size);
  if (comm_.write(pBuffer, size+1) != (int32_t)(size + 1)) {
    throw std::runtime_error("Failed to send op code.");
  }
}

void Create2::test() {
  sendOpCode(OC_START, 0, NULL);
  sleep_for_sec(2);
  //sendOpCode(OC_FULL, 0, NULL);
}
