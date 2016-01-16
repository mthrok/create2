#include <unistd.h>
#include <cstring>
#include <stdexcept>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "create2_driver/types.hpp"
#include "create2_driver/serial.hpp"

#include <create2_msgs/RoombaSensors.h>

namespace create2 {
  class Status;
  class Command;
  class SerialReader;
  class SerialWriter;
  class Communicator;
  class Create2;

  void swap(Command& c1, Command& c2);
};

class create2::Status {
  create2_msgs::RoombaSensors status_;
public:
  void update(uint8_t rawStatus[80]);
  std::string toString() const;
  create2_msgs::RoombaSensors getStatus() const;
};

class create2::SerialReader {
  Serial& serial_;
  boost::thread thread_;
  boost::atomic<bool> running_;
  Status status_;
  boost::mutex status_mutex_;

  uint32_t total, success, body_error, checksum_error;  // For debugging

  bool updateStatus();
  void updateStatusContinuously();
public:
  SerialReader(Serial& serial);
  ~SerialReader();

  void start();
  void stop();

  create2_msgs::RoombaSensors status();
};

class create2::Command {
  std::vector<uint8_t> data_;
public:
  Command(const OPCODE code,
          const uint8_t* data=NULL,
          const uint32_t datasize=0);

  const uint8_t* data() const;
  uint32_t size() const;
};

class create2::SerialWriter {
  Serial& serial_;
  boost::thread thread_;
  boost::atomic<bool> running_;
  std::deque<Command> commands_;
  boost::mutex command_mutex_;

  void processCommand(const Command& command);
  bool processNextCommand();
  void processCommandContinuously();
public:
  SerialWriter(Serial& serial);
  ~SerialWriter();

  void start();
  void stop();

  void queueCommand(const OPCODE code,
                    const uint8_t* data=NULL,
                    const uint32_t datasize=0);

  void clearCommands();
  void flushCommands();
};

class create2::Communicator {
  Serial serial_;
  SerialWriter writer_;
  SerialReader reader_;
public:
  Communicator();
  ~Communicator();

  void init(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");
  void start();
  void stop();

  void queueCommand(const OPCODE code,
                    const uint8_t* data=NULL,
                    const uint32_t size=0);
  void queueCommand(const OPCODE code,
                    const uint8_t data);

  void clearCommands();
  void flushCommands();

  create2_msgs::RoombaSensors getStatus();
};

class create2::Create2 {
  Communicator comm_;
public:
  Create2();
  ~Create2();

  void init(const int baudrate=B115200,
            const char* device="/dev/ttyUSB0");

  void start();
  void stop(bool block=false);
  void reset();

  void startStream();
  void stopStream();

  void passive();
  void safe();
  void full();

  void drive(const short velocity, const short radius=0);
  void driveDirect(const short right, const short left);

  create2_msgs::RoombaSensors getStatus();

  void test();
  void testDrive();
  void testStream();
  void testRestart();

};
