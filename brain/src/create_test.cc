#include "gtest/gtest.h"

#include "irobot-create.h"
#include <sstream>

namespace create {
namespace {
using::std::string;
using::std::stringstream;

TEST(CreateTest, StartSentOnCreation) {
  stringstream stream;
  Create robot(stream);
  EXPECT_EQ(stream.str(), string(1, static_cast<char>(Create::OPCODE_START)));
}

TEST(CreateTest, SampleCommandsSentCorrectly) {
  stringstream stream;
  Create robot(stream);
  char result[] = {
    static_cast<char>(128), //Start command
    static_cast<char>(129), //Baud rate command
    static_cast<char>(1), //baud 600 value
    0
  };
  robot.sendBaudCommand(Create::BAUD_600);
  EXPECT_EQ(stream.str(), result);
}
} //namespace
} //namespace create
