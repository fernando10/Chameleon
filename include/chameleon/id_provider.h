// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include <stdint.h>
#include <iostream>
#include <atomic>

namespace chameleon {

namespace IdGenerator
{
struct Instance
{
  static uint64_t NewId();
  static uint64_t CurrentId();
};
}  // namespace IdGenerator
} // namespace chameleon
