#include <unistd.h>
#include <cstring>
#include <stdexcept>

#include "create2_driver/types.hpp"
#include "create2_driver/serial.hpp"


using namespace create2;

class Create2 {
  Serial comm_;

public:
  Create2();
  ~Create2();

  void init(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");

  void sendOpCode(const OPCODE code, const uint32_t size, const uint8_t* pData);

  void test();
};
