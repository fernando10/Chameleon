// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/id_provider.h"

namespace chameleon
{
namespace IdGenerator
{
std::atomic<uint64_t> _id(0);
std::vector<uint64_t> _reserved_ids;
uint64_t Instance::NewId()
{
    uint64_t new_id = ++_id;
    for (const auto id : _reserved_ids) {
      if (new_id == id) {
        new_id = ++_id;
      }
    }
    return new_id;
}
uint64_t Instance::CurrentId()
{
    return _id;
}

bool Instance::ReserveIds(std::vector<uint64_t> ids_to_reserve) {
  for (const auto id : ids_to_reserve) {
    if (CurrentId() >= id) {
      return false;
    } else{
      _reserved_ids.push_back(id);
    }
  }
  return true;
}
}  // namespace IdGenerator
} // namespace chameleon

