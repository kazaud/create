/*
 * irobot_create_enums.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: jkobe
 */

#include "irobot_create_open_interface.h"

namespace create {
OpenInterface::OpenInterface() {
  //TODO Auto-generated constructor stub
}

OpenInterface::~OpenInterface() {
  //TODO Auto-generated destructor stub
}

const map<OpenInterface::Baud, LibSerial::SerialStreamBuf::BaudRateEnum> OpenInterface::kBaudMap = {
  {BAUD_300, LibSerial::SerialStreamBuf::BAUD_300},
  {BAUD_600, LibSerial::SerialStreamBuf::BAUD_600},
  {BAUD_1200, LibSerial::SerialStreamBuf::BAUD_1200},
  {BAUD_2400, LibSerial::SerialStreamBuf::BAUD_2400},
  {BAUD_4800, LibSerial::SerialStreamBuf::BAUD_4800},
  {BAUD_9600, LibSerial::SerialStreamBuf::BAUD_9600},
  {BAUD_19200, LibSerial::SerialStreamBuf::BAUD_19200},
  {BAUD_38400, LibSerial::SerialStreamBuf::BAUD_38400},
  {BAUD_57600, LibSerial::SerialStreamBuf::BAUD_57600},
  {BAUD_115200, LibSerial::SerialStreamBuf::BAUD_115200}
};
} /* namespace create */