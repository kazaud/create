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

unique_ptr<SerialStream> GetCreatesStream() {
  unique_ptr<SerialStream> virtual_create_port = MakeUnique<SerialStream>(
    "/home/jkobe/git/create/brain/build/virtual_create_serial_port",
    SerialStreamBuf::BAUD_57600, SerialStreamBuf::CHAR_SIZE_8,
    SerialStreamBuf::PARITY_NONE, 1, SerialStreamBuf::FLOW_CONTROL_NONE);
  EXPECT_NE(nullptr, virtual_create_port);
  EXPECT_TRUE(virtual_create_port->IsOpen());
  return virtual_create_port;
}

TEST(CreateTest, BadSerialPort) {
  EXPECT_EQ(nullptr, Create::MakeFromPort("ThisIsABogusSerialPortName"));
}

TEST(CreateTest, LoopbackPortCreated) {
  unique_ptr<SerialStream> virtual_create_port = GetCreatesStream();
  EXPECT_NE(nullptr, virtual_create_port);
  EXPECT_TRUE(virtual_create_port->IsOpen());
  EXPECT_NE(
    nullptr,
    Create::MakeFromPort(
      "/home/jkobe/git/create/brain/build/virtual_linux_serial_port"));
  EXPECT_EQ(virtual_create_port->peek(), OpenInterface::OPCODE_START);
}
} //namespace
} //namespace create
