#include "create_config.h"
#include "irobot_create.h"
#include <SerialStream.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include "tools/using_std_stuff.h"

using ::std::stringstream;
using namespace LibSerial;
using namespace create;

int main(int argc, char* argv[]) {
  fprintf(stdout, "Create version %d.%d\n", kCreateVersionMajor, kCreateVersionMinor);
  if (argc < 2) {
    fprintf(stdout, "Usage: %s /dev/ttyUSB0\n", argv[0]);
    return 1;
  }

  char input;
  unique_ptr < Create > create = Create::MakeFromPort(argv[1]);
  create->sendSafeCommand();
  while (true) {
    //create->
    cout << "Turning right" << endl;
    create->sendDriveCommand(100, 1);
    cin >> input;
    cout << "Turning left" << endl;
    create->sendDriveCommand(-100, 1);
    cin >> input;
  }
  return 0;
}
