// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <stdint.h>
#include <iostream>
#include <atomic>
#include <vector>

namespace chameleon {

namespace IdGenerator
{
struct Instance
{
  static uint64_t NewId();
  static uint64_t CurrentId();
  static bool ReserveIds(std::vector<uint64_t> ids_to_reserve);
};
}  // namespace IdGenerator
} // namespace chameleon
