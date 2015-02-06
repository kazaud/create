#ifndef TOOLS_USING_STD_STUFF_H_
#define TOOLS_USING_STD_STUFF_H_

#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <cstdlib>

using ::std::cout;
using ::std::cin;
using ::std::endl;
using ::std::initializer_list;
using ::std::iostream;
using ::std::map;
using ::std::pair;
using ::std::queue;
using ::std::string;
using ::std::unique_ptr;
using ::std::vector;

class StringLikeThings {
public:
  template<typename StringLikeThing>
  static string ToString(StringLikeThing thing) {
    return std::to_string(thing);
  }
  static string ToString(const char* thing){
    return string(thing);
  }
};

template<typename Arg1, typename Arg2, typename Arg3, typename Arg4>
string StrCat(Arg1 s1, Arg2 s2, Arg3 s3, Arg4 s4) {
  string value = StringLikeThings::ToString(s1);
  value += StringLikeThings::ToString(s2);
  value += StringLikeThings::ToString(s3);
  value += StringLikeThings::ToString(s4);
  return std::move(value);
}

template<typename Arg1, typename Arg2, typename Arg3>
string StrCat(Arg1 s1, Arg2 s2, Arg3 s3) {
  string value = StringLikeThings::ToString(s1);
  value += StringLikeThings::ToString(s2);
  value += StringLikeThings::ToString(s3);
  return std::move(value);
}

#endif //TOOLS_USING_STD_STUFF_H_
