#include "create_config.h"
#include "irobot-create.h"
#include <SerialStream.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

using::std::stringstream;
using namespace LibSerial;
using namespace create;

int main(int argc, char* argv[]) {
  fprintf(stdout, "Create version %d.%d\n", kCreateVersionMajor, kCreateVersionMinor);
  if (argc < 2) {
    fprintf(stdout, "Usage: %s serial port. I.e. /dev/ttyUSB0\n", argv[0]);
    return 1;
  }
  SerialStream stream("/dev/ttyS0", SerialStreamBuf::BAUD_57600);
  Create robot(stream);
  return 0;
}