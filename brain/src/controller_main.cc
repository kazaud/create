#include "create_config.h"
#include "irobot_create.h"
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
    fprintf(stdout, "Usage: %s /dev/ttyUSB0\n", argv[0]);
    return 1;
  }
  SerialStream stream(argv[1], SerialStreamBuf::BAUD_57600);
  stream << static_cast<char>(128);
  stream << static_cast<char>(136);
  stream << static_cast<char>(9);

//  unique_ptr<Create> create = Create::MakeFromPort(argv[1]);
//    create->sendSafeCommand();
//    create->sendDriveCommand(10, 100);
//    sleep(3);
//    create->sendDriveCommand(10, -100);
//    sleep(3);
  return 0;
}
