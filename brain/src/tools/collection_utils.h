/*
 * make_unique.h
 *
 *  Created on: Feb 3, 2015
 *      Author: jkobe
 */

#ifndef COLLECTION_UTILS_H_
#define COLLECTION_UTILS_H_

#include "tools/using_std_stuff.h"

template<typename Collection, typename Value>
bool ContainsKey(const Collection& collection, const Value& value) {
  return collection.find(value) != collection.end();
}

#endif //COLLECTION_UTILS_H_