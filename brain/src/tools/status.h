#ifndef STATUS_H_
#define STATUS_H_

#include "tools/using_std_stuff.h"

#define RETURN_IF_ERROR(STATUS)\
  {\
    Status status = STATUS;\
    if (!status.ok()) {\
      return status;\
    }\
  }

#define CHECK_OK(STATUS)\
{ \
  Status status = STATUS; \
  if (!status.ok()) { \
    std::cout << "Status is not okay on " << __FILE__ << ":" \
              << __LINE__ << "!\nerror:" << status.error(); \
  } \
}

class Status {
public:
  static Status Ok;
  enum Code {
    CODE_OK, CODE_ERROR
  };
  Status(const Status& other)
      : code_(other.code_), error_(other.error_) {
  }
  Status(const string& error)
      : code_(CODE_ERROR), error_(error) {
  }
  Status(const char* error)
      : code_(CODE_ERROR), error_(error) {
  }
  bool ok() const {
    return code_ == CODE_OK;
  }
  const string & error() const {
    return error_;
  }
private:
  Status()
      : code_(CODE_OK) {
  }
  const Code code_;
  const string error_;
};

#endif/* STATUS_H_ */
