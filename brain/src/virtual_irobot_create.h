//An irobot create that can be run completely in software without the hardward.

#ifndef VIRTUAL_IROBOT_CREATE_H_
#define VIRTUAL_IROBOT_CREATE_H_

#include "irobot_create.h"

namespace create {
class VirtualIrobotCreate: public Create {
public:
  VirtualIrobotCreate();
  virtual ~VirtualIrobotCreate();
};
} /* namespace create */

#endif/* VIRTUAL_IROBOT_CREATE_H_ */