#include "irobot_create.h"

#include "gtest/gtest.h"
#include "tools/make_unique.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <unistd.h>

namespace create {
namespace {
using ::LibSerial::SerialStream;
using ::LibSerial::SerialStreamBuf;

class CreateTest : public ::testing::Test {
protected:
  void SetUp() override {
    virtual_create_port_ = MakeUnique<SerialStream>(
        "/home/jkobe/git/create/brain/build/virtual_create_serial_port",
        SerialStreamBuf::BAUD_57600, SerialStreamBuf::CHAR_SIZE_8,
        SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);
    EXPECT_NE(nullptr, virtual_create_port_);
    EXPECT_TRUE(virtual_create_port_->IsOpen());
  }
  void TearDown() override {
    // Make sure test verified all of the data sent.
    EXPECT_EQ(0, virtual_create_port_->rdbuf()->in_avail());
  }
  void ExpectData(const initializer_list<int>& data_list) {
    int byte_count = 0;
    for (int data : data_list) {
      int timeout = 1000;
      while (virtual_create_port_->rdbuf()->in_avail() == 0 && timeout > 0) {
        timeout -= 50;
        usleep(50);
      }
      ASSERT_TRUE(virtual_create_port_->rdbuf()->in_avail())<< "Timed out while waiting for byte " << byte_count << " of value " << data;
      char next_byte;
      virtual_create_port_->get(next_byte);
      EXPECT_EQ(static_cast<char>(data), next_byte) << "Byte " << byte_count
          << " was not as expected!";
      ++byte_count;
    }
  }

  const char* kSerialPort =
      "/home/jkobe/git/create/brain/build/virtual_linux_serial_port";
  unique_ptr<SerialStream> virtual_create_port_;
};

TEST_F(CreateTest, BadSerialPort) {
  EXPECT_EQ(nullptr, Create::MakeFromPort("ThisIsABogusSerialPortName"));
}

TEST_F(CreateTest, LoopbackPortCreated) {
  EXPECT_NE(nullptr, Create::MakeFromPort(kSerialPort));
  ExpectData(
    {OpenInterface::OPCODE_START});
}

TEST_F(CreateTest, SetBaudRateWorks) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  ExpectData(
    {OpenInterface::OPCODE_START});
  for (const auto& pair : OpenInterface::kBaudMap) {
    if (robot->sendBaudCommand(pair.first)) {
      ExpectData(
        {OpenInterface::OPCODE_BAUD, pair.first});
      virtual_create_port_->SetBaudRate(pair.second);
    }
  }
}

TEST_F(CreateTest, CanChangeModes) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  EXPECT_FALSE(robot->sendStartCommand());
  EXPECT_EQ(OpenInterface::MODE_PASSIVE, robot->mode());
  EXPECT_TRUE(robot->sendFullCommand());
  EXPECT_FALSE(robot->sendFullCommand());
  EXPECT_EQ(OpenInterface::MODE_FULL, robot->mode());
  EXPECT_TRUE(robot->sendSafeCommand());
  EXPECT_FALSE(robot->sendSafeCommand());
  EXPECT_EQ(OpenInterface::MODE_SAFE, robot->mode());
  EXPECT_TRUE(robot->sendStartCommand());
  EXPECT_EQ(OpenInterface::MODE_PASSIVE, robot->mode());

  ExpectData(
    {OpenInterface::OPCODE_START, OpenInterface::OPCODE_FULL,
        OpenInterface::OPCODE_SAFE, OpenInterface::OPCODE_START});
}

TEST_F(CreateTest, DriveCommand_WithRadius) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  EXPECT_FALSE(robot->sendDriveCommand(2, 2));
  EXPECT_TRUE(robot->sendFullCommand());
  EXPECT_TRUE(robot->sendDriveCommand(2, 2));
  ExpectData(
    {OpenInterface::OPCODE_START, OpenInterface::OPCODE_FULL,
        OpenInterface::OPCODE_DRIVE, 0x0000, 0x0002, 0x0000, 0x0002});
}

TEST_F(CreateTest, DriveCommand_WithCommand) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  EXPECT_FALSE(
      robot->sendDriveCommand(2, OpenInterface::DriveCommand::DRIVE_STRAIGHT));
  EXPECT_TRUE(robot->sendFullCommand());
  EXPECT_TRUE(
      robot->sendDriveCommand(2, OpenInterface::DriveCommand::DRIVE_STRAIGHT));
  EXPECT_TRUE(
      robot->sendDriveCommand(2,
                              OpenInterface::DriveCommand::DRIVE_INPLACE_CLOCKWISE));
  EXPECT_TRUE(
      robot->sendDriveCommand(
          2, OpenInterface::DriveCommand::DRIVE_INPLACE_COUNTERCLOCKWISE));
  ExpectData(
    {OpenInterface::OPCODE_START, OpenInterface::OPCODE_FULL,
        OpenInterface::OPCODE_DRIVE, 0x00, 0x02, 0x80, 0x00,
        OpenInterface::OPCODE_DRIVE, 0x00, 0x02, 0xFF, 0xFF,
        OpenInterface::OPCODE_DRIVE, 0x00, 0x02, 0x00, 0x01});
}

TEST_F(CreateTest, DriveDirectCommand) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  EXPECT_FALSE(robot->sendDriveDirectCommand(2, 2));
  EXPECT_TRUE(robot->sendFullCommand());
  EXPECT_TRUE(robot->sendDriveDirectCommand(2, 2));
  EXPECT_TRUE(robot->sendDriveDirectCommand(4, 4));
  ExpectData(
    {OpenInterface::OPCODE_START, OpenInterface::OPCODE_FULL,
        OpenInterface::OPCODE_DRIVE_DIRECT, 0x00, 0x02, 0x00, 0x02,
        OpenInterface::OPCODE_DRIVE_DIRECT, 0x00, 0x04, 0x00, 0x04});
}

TEST_F(CreateTest, LedCommand) {
  unique_ptr<Create> robot = Create::MakeFromPort(kSerialPort);
  EXPECT_FALSE(
      robot->sendLedCommand(OpenInterface::LED_PLAY, OpenInterface::LED_COLOR_GREEN,
                            OpenInterface::LED_INTENSITY_FULL));
  EXPECT_TRUE(robot->sendFullCommand());
  EXPECT_TRUE(
      robot->sendLedCommand(OpenInterface::LED_PLAY, OpenInterface::LED_COLOR_GREEN,
                            OpenInterface::LED_INTENSITY_FULL));
  ExpectData(
    {OpenInterface::OPCODE_START, OpenInterface::OPCODE_FULL,
        OpenInterface::OPCODE_LEDS, OpenInterface::LED_PLAY,
        OpenInterface::LED_COLOR_GREEN, OpenInterface::LED_INTENSITY_FULL});
}
}  //namespace
}  //namespace create
