#include <iostream>
#include "create2_driver/create2_driver.hpp"

void sleep_for_sec (float sec) {
  long usec=(long)(sec*1000000);
  usleep(usec);
}

create2::Communicator::Communicator()
  : serial_()
{}

create2::Communicator::~Communicator()
{};

void create2::Communicator::init(
    const int baudrate, const char* device) {
  serial_.init(baudrate, device);
}

void create2::Communicator::sendOpCode(
    const OPCODE code, const uint8_t* pData, const uint32_t size) {
  uint8_t* pBuffer = new uint8_t[size + 1];
  pBuffer[0] = (uint8_t)code;
  memcpy(pBuffer+1, pData, size);
  if (serial_.write(pBuffer, size+1) != (int32_t)(size + 1)) {
    std::cerr << "Failed to send Opcode: " << code << std::endl;
  }
  delete[] pBuffer;
}

void create2::Communicator::sendOpCode(const OPCODE code) {
  uint8_t c = (uint8_t)code;
  if (serial_.write(&c, 1) != 1) {
    std::cerr << "Failed to send Opcode: " << code << std::endl;
  }
}

create2::Create2::Create2()
  : comm_()
{};

create2::Create2::~Create2() {};

void create2::Create2::init(const int baudrate, const char* device) {
  comm_.init(baudrate, device);
};

void create2::Create2::test() {
  comm_.sendOpCode(OC_START);
  sleep_for_sec(1);

  std::cout << "[TEST] Entering FULL mode." << std::endl;
  comm_.sendOpCode(OC_FULL);
  sleep_for_sec(1);

  std::cout << "[TEST] Entering PASSIVE mode." << std::endl;
  comm_.sendOpCode(OC_START);
  sleep_for_sec(1);

  std::cout << "[TEST] Restarting." << std::endl;
  comm_.sendOpCode(OC_RESET);
  sleep_for_sec(5);
}
