#include <iostream>
#include "create2_driver/create2_driver.hpp"

void sleep_for_sec (float sec) {
  long usec=(long)(sec*1000000);
  usleep(usec);
}

uint16_t convertUShort(const uint8_t packet1, const uint8_t packet2) {
  return (uint16_t)((packet1<<8)|packet2);
}

int16_t convertShort(const uint8_t packet1, const uint8_t packet2) {
  return (int16_t)((packet1<<8)|packet2);
}

void create2::Status::update(uint8_t rawStatus[80]) {
  // Bumps and Wheel Drops
  status_.bumps_wheeldrops.bump_right      = (rawStatus[0] & BUMP_RIGHT);
  status_.bumps_wheeldrops.bump_left       = (rawStatus[0] & BUMP_LEFT);
  status_.bumps_wheeldrops.wheeldrop_right = (rawStatus[0] & WHEELDROP_RIGHT);
  status_.bumps_wheeldrops.wheeldrop_left  = (rawStatus[0] & WHEELDROP_LEFT);
  // Wall
  status_.wall.wall                        = (rawStatus[1] & 0x01);
  // Cliff
  status_.cliff.left                       = (rawStatus[2] & 0x01);
  status_.cliff.front_left                 = (rawStatus[3] & 0x01);
  status_.cliff.front_right                = (rawStatus[4] & 0x01);
  status_.cliff.right                      = (rawStatus[5] & 0x01);
  // Vertial Wall
  status_.wall.vwall                       = (rawStatus[6] & 0x01);
  // Wheel Overcurrents
  status_.wheel_overcurrents.left_wheel    = (rawStatus[7] & LEFT_WHEEL);
  status_.wheel_overcurrents.right_wheel   = (rawStatus[7] & RIGHT_WHEEL);
  status_.wheel_overcurrents.main_brush    = (rawStatus[7] & MAIN_BRUSH);
  status_.wheel_overcurrents.side_brush    = (rawStatus[7] & SIDE_BRUSH);
  // Dirt Detect
  status_.dirt_detect                      = rawStatus[8];
  // Unused Byte
  // rawStatus[9] is unused byte
  // IR Opcodes
  status_.ir_opcodes.omni                  = rawStatus[10];
  status_.ir_opcodes.left                  = rawStatus[69]; // <- Attention
  status_.ir_opcodes.right                 = rawStatus[70]; // <- It's confusing.
  // Buttons
  status_.button.clean                     = (rawStatus[11] & BB_CLEAN);
  status_.button.spot                      = (rawStatus[11] & BB_SPOT);
  status_.button.dock                      = (rawStatus[11] & BB_DOCK);
  status_.button.minute                    = (rawStatus[11] & BB_MINUTE);
  status_.button.hour                      = (rawStatus[11] & BB_HOUR);
  status_.button.day                       = (rawStatus[11] & BB_DAY);
  status_.button.schedule                  = (rawStatus[11] & BB_SCHEDULE);
  status_.button.clock                     = (rawStatus[11] & BB_CLOCK);
  // Traveled distance and angle
  status_.travel.distance                  = convertShort(rawStatus[12], rawStatus[13]);
  status_.travel.angle                     = convertShort(rawStatus[14], rawStatus[15]);
  // Battery Charging status_, Voltage, Current, Temperature, Charge, Capacity
  status_.battery.charging_state           = rawStatus[16];
  status_.battery.voltage                  = convertUShort(rawStatus[17], rawStatus[18]);
  status_.battery.current                  = convertShort(rawStatus[19], rawStatus[20]);
  status_.battery.temperature              = *((int8_t*)   (rawStatus + 21));
  status_.battery.charge                   = convertUShort(rawStatus[22], rawStatus[23]);
  status_.battery.capacity                 = convertUShort(rawStatus[24], rawStatus[25]);
  // Wall Signal Strength
  status_.wall.wall_signal                 = convertUShort(rawStatus[26], rawStatus[27]);
  // Cliff Signal Strength
  status_.cliff.left_signal                = convertUShort(rawStatus[28], rawStatus[29]);
  status_.cliff.front_left_signal          = convertUShort(rawStatus[30], rawStatus[31]);
  status_.cliff.front_right_signal         = convertUShort(rawStatus[32], rawStatus[33]);
  status_.cliff.right_signal               = convertUShort(rawStatus[34], rawStatus[35]);
  // Unused Byte
  // rawStatus[36, 37, 38] is unused byte
  // Charging sources availability
  status_.charging_source.home_base        = (rawStatus[39] & HOME_BASE);
  status_.charging_source.internal_charger = (rawStatus[39] & INTERNAL_CHARGER);
  // OI Mode
  status_.oi_mode                          = rawStatus[40];
  // Song
  status_.song.number                      = rawStatus[41];
  status_.song.playing                     = rawStatus[42];
  // #Stream rawStatusets
  status_.stream_packets                   = rawStatus[43];
  // Requested Velocity and radius
  status_.request.velocity                 = convertShort(rawStatus[44], rawStatus[45]);
  status_.request.radius                   = convertShort(rawStatus[46], rawStatus[47]);
  status_.request.right_velocity           = convertShort(rawStatus[48], rawStatus[49]);
  status_.request.left_velocity            = convertShort(rawStatus[50], rawStatus[51]);
  // Encoder counts
  status_.encoder_counts.left              = convertShort(rawStatus[52], rawStatus[53]);
  status_.encoder_counts.right             = convertShort(rawStatus[54], rawStatus[55]);
  // Light Bumper
  status_.light_bumper.left                = (rawStatus[56] & LT_BUMPER_LEFT);
  status_.light_bumper.front_left          = (rawStatus[56] & LT_BUMPER_FRONT_LEFT);
  status_.light_bumper.center_left         = (rawStatus[56] & LT_BUMPER_CENTER_LEFT);
  status_.light_bumper.center_right        = (rawStatus[56] & LT_BUMPER_CENTER_RIGHT);
  status_.light_bumper.front_right         = (rawStatus[56] & LT_BUMPER_FRONT_RIGHT);
  status_.light_bumper.right               = (rawStatus[56] & LT_BUMPER_RIGHT);
  // Light Bumper Signal Strength
  status_.light_bumper.left_signal         = convertUShort(rawStatus[57], rawStatus[58]);
  status_.light_bumper.front_left_signal   = convertUShort(rawStatus[59], rawStatus[60]);
  status_.light_bumper.center_left_signal  = convertUShort(rawStatus[61], rawStatus[62]);
  status_.light_bumper.center_right_signal = convertUShort(rawStatus[63], rawStatus[64]);
  status_.light_bumper.front_right_signal  = convertUShort(rawStatus[65], rawStatus[66]);
  status_.light_bumper.right_signal        = convertUShort(rawStatus[67], rawStatus[68]);
  // Motor Current
  status_.motor_current.left_wheel         = convertShort(rawStatus[71], rawStatus[72]);
  status_.motor_current.right_wheel        = convertShort(rawStatus[73], rawStatus[74]);
  status_.motor_current.main_brush         = convertShort(rawStatus[75], rawStatus[76]);
  status_.motor_current.side_brush         = convertShort(rawStatus[77], rawStatus[78]);
  // Stasis
  status_.stasis                           =  rawStatus[79];
}

std::string create2::Status::toString() const {
  std::stringstream ss;
  ss << "\nBumps:\n  "
     << (status_.bumps_wheeldrops.bump_right ? "Right, " : "       ")
     << (status_.bumps_wheeldrops.bump_left  ? "Left" : "")
     << "\nWheeldrops:\n "
     << (status_.bumps_wheeldrops.wheeldrop_right ? "Right, " : "       ")
     << (status_.bumps_wheeldrops.wheeldrop_left  ? "Left" : "")
     << "\nWall:\n  "
     << (status_.wall.wall ? "Wall, " : "      ")
     << (status_.wall.vwall ? "Virtual Wall" : "")
     << "\nCliff:\n  "
     << (status_.cliff.left ? "Left, " : "")
     << (status_.cliff.front_left ? "Front Left, " : "")
     << (status_.cliff.right ? "Right, " : "")
     << (status_.cliff.front_right ? "Front Right " : "")
     << "\nWheel Overcurrent:\n  "
     << (status_.wheel_overcurrents.side_brush ? "Side Brush, " : "")
     << (status_.wheel_overcurrents.main_brush ? "Main Brush, " : "")
     << (status_.wheel_overcurrents.right_wheel ? "Right Wheel, " : "")
     << (status_.wheel_overcurrents.left_wheel ? "Left Wheel" : "")
     << "\nDirt Detection:  " << (int)status_.dirt_detect
     << "\nIR Character:"
     << "\n  Right: " << (int)status_.ir_opcodes.right
     << "\n  Left: "  << (int)status_.ir_opcodes.left
     << "\n  Omni: "  << (int)status_.ir_opcodes.omni
     << "\nButtons:\n  "
     << (status_.button.clean    ? "Clean, " : "")
     << (status_.button.spot     ? "Spot, " : "")
     << (status_.button.dock     ? "Dock, " : "")
     << (status_.button.minute   ? "Minute, " : "")
     << (status_.button.hour     ? "Hour, " : "")
     << (status_.button.day      ? "Day, " : "")
     << (status_.button.clock    ? "Clock, " : "")
     << (status_.button.schedule ? "Schedule" : "")
     << "\nTraveled (From the last acquisition.):"
     << "\n  Distance: " << status_.travel.distance << " [mm]"
     << "\n  Angle: " << status_.travel.angle << " [deg]"
     << "\nBattery:"
     << "\n  Charging State: " << (uint)status_.battery.charging_state
     << "\n  Voltage: " << status_.battery.voltage
     << "\n  Current: " << status_.battery.current
     << "\n  Temperature: " << (int) status_.battery.temperature
     << "\n  Ramains: "
     << status_.battery.charge << " / " << status_.battery.capacity
     << " ("
     << 100.0f * status_.battery.charge / status_.battery.capacity
     << "[%])"
     << "\nCharge Source:\n  "
     << (status_.charging_source.home_base ? "Home Base, " : "")
     << (status_.charging_source.internal_charger ? "Internal Charger." : "")
     << "\nOI Mode : " << (int)status_.oi_mode
     << "\nSong:"
     << "\n  Numer:   " << (int)status_.song.number
     << "\n  Playing: " << (int)status_.song.playing
     << "\nRequest:"
     << "\n  Velocity: " << status_.request.velocity
     << "\n  Radius: " << status_.request.radius
     << "\n  Right Velocity: " << status_.request.right_velocity
     << "\n  Left Velocity: " << status_.request.left_velocity
     << "\nEncoder Count:"
     << "\n  L / R: " << status_.encoder_counts.right
     << " / " << status_.encoder_counts.left
     << "\nLight Bumper: "
     << "\n  Left:         "
     << (status_.light_bumper.left ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.left_signal << ")"
     << "\n  Right:        "
     << (status_.light_bumper.right ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.right_signal << ")"
     << "\n  Front Left:   "
     << (status_.light_bumper.front_left ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.front_left_signal << ")"
     << "\n  Front Right:  "
     << (status_.light_bumper.front_right ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.front_right_signal << ")"
     << "\n  Center Left:  "
     << (status_.light_bumper.center_left ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.center_left_signal << ")"
     << "\n  Center Right: "
     << (status_.light_bumper.center_right ? "    Hit" : "Not Hit")
     << ", (" << status_.light_bumper.center_right_signal << ")"
     << "\nMotor Current:"
     << "\n  Left Wheel: " << status_.motor_current.left_wheel
     << "\n  Right Wheel: " << status_.motor_current.right_wheel
     << "\n  Main Brush: " << status_.motor_current.main_brush
     << "\n  Side Brush: " << status_.motor_current.side_brush
     << "\nStasis: " << (status_.stasis ? "Forward" : "Not Forward")
     << "\n";
  return ss.str();
}

create2_msgs::RoombaSensors create2::Status::getStatus() const {
  return status_;
}

create2::SerialReader::SerialReader(Serial& serial)
  : serial_(serial)
  , thread_()
  , running_(false)
  , status_()
  , status_mutex_()
  , total(0)
  , success(0)
  , body_error(0)
  , checksum_error(0)
{}

create2::SerialReader::~SerialReader() {
  stop();
  std::cout
    << "SreialReader Summary:\n"
    << "  ERROR(  Body  ): " << body_error << "\n"
    << "  ERROR(Checksum): " << checksum_error << "\n"
    << "          SUCCESS: " << success << "\n"
    << "            TOTAL: " << total << std::endl;
};

void create2::SerialReader::start() {
  if (!running_) {
    std::cout << "Starting SerialReader." << std::endl;
    running_ = true;
    thread_ = boost::thread(
      &create2::SerialReader::updateStatusContinuously, this);
    std::cout << "SerialReader started." << std::endl;
  }
};

void create2::SerialReader::stop() {
  if (running_) {
    std::cout << "Waiting for SerialReader to stop." << std::endl;
    running_ = false;
    thread_.join();
    std::cout << "SerialReader stopped." << std::endl;
  }
}

create2_msgs::RoombaSensors create2::SerialReader::status() {
  boost::lock_guard<boost::mutex> lock(status_mutex_);
  return status_.getStatus();
}

bool create2::SerialReader::updateStatus() {
  int read;
  uint8_t header, size, checksum;
  read = serial_.read(&header, 1);
  if (!(read == 1 && header == 19)) {
    return false;
  }
  ++total;
  if (1 != serial_.read(&size, 1)) {
    return false;
  }
  std::vector<uint8_t> body(size);
  if (size != serial_.read(&body.front(), size)) {
    ++body_error;
    return false;
  }
  if (1 != serial_.read(&checksum, 1)) {
    return false;
  }
  checksum += header + size;
  for (uint8_t i = 0; i < size; ++i) {
    checksum += body[i];
  }
  if (checksum != 0) {
    ++checksum_error;
    return false;
  }
  for (uint8_t i = 0; i < size; ) {
    int packetId = body[i++];
    switch(packetId) {
    case 100:
      status_.update(&body[i]);
      i += 80;
      ++success;
      // std::cout << "*" << std::flush;
      // std::cout << status_.toString() << std::flush;
      break;
    default:
      std::cerr << "NOT IMPLEMENTED: " << packetId << "\n";
    }
  }
  return true;
}

void create2::SerialReader::updateStatusContinuously() {
  boost::chrono::milliseconds interval(3);
  while(running_) {
    bool success = status_mutex_.try_lock();
    if (success) {
      success = updateStatus();
      status_mutex_.unlock();
    }
    boost::this_thread::sleep_for(success ? 5 * interval : interval);
  }
}

create2::Command::Command(const OPCODE code, const uint8_t* data, const uint32_t datasize)
  : data_(datasize + 1)
{
  data_[0] = (uint8_t) code;
  memcpy(&data_[1], data, datasize);
}

const uint8_t* create2::Command::data() const {
  return &data_[0];
}

uint32_t create2::Command::size() const {
  return data_.size();
}

create2::SerialWriter::SerialWriter(Serial& serial)
  : serial_(serial)
  , thread_()
  , running_()
  , commands_()
{}

create2::SerialWriter::~SerialWriter() {
  stop();
};

void create2::SerialWriter::start() {
  if (!running_) {
    std::cout << "Starting SerialWriter." << std::endl;
    running_ = true;
    thread_ = boost::thread(
      &create2::SerialWriter::processCommandContinuously, this);
    std::cout << "SerialWriter started." << std::endl;
  }
}

void create2::SerialWriter::stop() {
  if (running_) {
    std::cout << "Waiting for SerialWriter to stop." << std::endl;
    running_ = false;
    thread_.join();
    std::cout << "SerialWriter stopped." << std::endl;
  }
}

void create2::SerialWriter::queueCommand(
    const OPCODE code, const uint8_t* data, const uint32_t datasize) {
  boost::lock_guard<boost::mutex> lock(command_mutex_);
  commands_.push_back(Command(code, data, datasize));
};

void create2::SerialWriter::clearCommands() {
  boost::lock_guard<boost::mutex> lock(command_mutex_);
  commands_.clear();
}

void create2::SerialWriter::processCommand(const Command& command) {
  uint32_t size = command.size();
  const uint8_t* data = command.data();
  std::cout << "command: " << (int)data[0] << std::endl;
  if (serial_.write(data, size) != (int32_t) size) {
    std::cerr << "Failed to process: " << (int)data[0] << std::endl;
  }
};

bool create2::SerialWriter::processNextCommand() {
  // Make sure that command_mutes_ is locked by caller thread.
  if (commands_.size()) {
    processCommand(commands_[0]);
    commands_.pop_front();
    return true;
  }
  return false;
}

void create2::SerialWriter::flushCommands() {
  boost::lock_guard<boost::mutex> lock(command_mutex_);
  while(commands_.size()) {
    processNextCommand();
  }
}

void create2::SerialWriter::processCommandContinuously() {
  boost::chrono::milliseconds interval(3);
  while(running_) {
    bool success = command_mutex_.try_lock();
    if (success) {
      success = processNextCommand();
      command_mutex_.unlock();
    }
    boost::this_thread::sleep_for(success? 5 * interval : interval);
  }
}

create2::Communicator::Communicator()
  : serial_()
  , writer_(serial_)
  , reader_(serial_)
{}

create2::Communicator::~Communicator() {
  stop();
};

void create2::Communicator::init(
    const int baudrate, const char* device) {
  serial_.open(baudrate, device);
};

void create2::Communicator::start() {
  writer_.start();
  reader_.start();
}

void create2::Communicator::stop() {
  writer_.stop();
  reader_.stop();
}

void create2::Communicator::queueCommand(
    const OPCODE code, const uint8_t* pData, const uint32_t size) {
  writer_.queueCommand(code, pData, size);
}

void create2::Communicator::queueCommand(
    const OPCODE code, const uint8_t data) {
  writer_.queueCommand(code, &data, 1);
};

void create2::Communicator::clearCommands() {
  writer_.clearCommands();
}

void create2::Communicator::flushCommands() {
  writer_.flushCommands();
}

create2::Create2::Create2()
  : comm_()
{};

create2::Create2::~Create2() {
  stop(true);
};

void create2::Create2::init(const int baudrate, const char* device) {
  comm_.init(baudrate, device);
  comm_.start();
};

void create2::Create2::startStream() {
  uint8_t data[] = {1, 100};
  comm_.queueCommand(OC_STREAM, data, 2);
}

void create2::Create2::stopStream() {
  comm_.queueCommand(OC_PAUSE_STREAM, (uint8_t)0);
}

void create2::Create2::drive(const short velocity, const short radius) {
  /**
     velocity: -500 ~ 500 [mm/s]
     radius: -2000 ~ 2000 [mm]
   **/
  uint8_t data[4] = {
    (uint8_t)(velocity >> 8),
    (uint8_t)(velocity & 0xff),
    (uint8_t)(radius >> 8),
    (uint8_t)(radius & 0xff)
  };
  comm_.queueCommand(OC_DRIVE, data, 4);
}

void create2::Create2::driveDirect(const short right, const short left) {
  /**
     right/left: -500 ~ 500 [mm/s]
   **/
  uint8_t data[4] = {
    (uint8_t)(right >> 8),
    (uint8_t)(right & 0xff),
    (uint8_t)(left >> 8),
    (uint8_t)(left & 0xff)
  };
  comm_.queueCommand(OC_DRIVE_DIRECT, data, 4);
}

void create2::Create2::passive() {
  comm_.queueCommand(OC_START);
}

void create2::Create2::safe() {
  comm_.queueCommand(OC_SAFE);
}

void create2::Create2::full() {
  comm_.queueCommand(OC_FULL);
}

void create2::Create2::start() {
  passive();
  startStream();
}

void create2::Create2::stop(bool block) {
  comm_.clearCommands();
  stopStream();
  passive();
  if (block) {
    comm_.flushCommands();
  }
}

void create2::Create2::reset() {
  comm_.queueCommand(OC_RESET);
}

void create2::Create2::testDrive() {
  std::cout << "[TEST] Testing drive." << std::endl;
  full();
  for (int i = 0; i < 120; ++i) {
    drive(30, 0);
  }
  for (int i = 0; i < 120; ++i) {
    drive(-30, 0);
  }
  for (int i = 0; i < 120; ++i) {
    driveDirect(-100, 100);
  }
  for (int i = 0; i < 120; ++i) {
    driveDirect(100, -100);
  }
  sleep_for_sec(4);
}

void create2::Create2::testStream() {
  std::cout << "[TEST] Testing Stream pause/resume." << std::endl;
  passive();

  std::cout << "[TEST] Start stream." << std::endl;
  startStream();
  sleep_for_sec(5);

  std::cout << "[TEST] Pause stream." << std::endl;
  stopStream();
  sleep_for_sec(5);

  std::cout << "[TEST] Resume stream." << std::endl;
  startStream();
  sleep_for_sec(5);
}

void create2::Create2::testRestart() {
  std::cout << "[TEST] Resetarting." << std::endl;
  reset();
  sleep_for_sec(3.5);
  start();
}

void create2::Create2::test() {
  start();
  // testStream();
  // testRestart();
  testDrive();
  stop();
}
