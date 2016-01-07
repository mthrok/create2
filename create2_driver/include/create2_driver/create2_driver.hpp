#include <unistd.h>
#include <cstring>
#include <stdexcept>

#include "create2_driver/types.hpp"
#include "create2_driver/serial.hpp"

using namespace create2;

namespace create2 {
  class Communicator;
  class Create2;
};

class create2::Communicator {
  Serial serial_;

public:
  Communicator();
  ~Communicator();

  void init(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");
  void start();

  void sendOpCode(const OPCODE code);
  void sendOpCode(const OPCODE code,
                  const uint8_t* pData,
                  const uint32_t size);

};

class create2::Create2 {
  Communicator comm_;

public:
  Create2();
  ~Create2();

  void init(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");

  void test();
};
