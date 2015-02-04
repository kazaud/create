/*
 * make_unique.h
 *
 *  Created on: Feb 3, 2015
 *      Author: jkobe
 */

#ifndef MAKE_UNIQUE_H_
#define MAKE_UNIQUE_H_

#include "tools/using_std_stuff.h"

template<typename T, typename... Args>
unique_ptr<T> make_unique_helper(std::false_type, Args && ... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args) ...));
}

template<typename T, typename... Args>
unique_ptr<T> make_unique_helper(std::true_type, Args && ... args) {
  static_assert(std::extent<T>::value == 0,
                "make_unique<T[N]>() is forbidden, please use make_unique<T[]>().");

  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[sizeof ... (Args)] {std::forward<Args>(args) ...});
}

template<typename T, typename... Args>
unique_ptr<T> MakeUnique(Args && ... args) {
  return make_unique_helper<T>(std::is_array<T>(), std::forward<Args>(args) ...);
}

#endif/* MAKE_UNIQUE_H_ */
