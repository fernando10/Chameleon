// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/id_provider.h"

namespace chameleon
{
namespace IdGenerator
{
std::atomic<uint64_t> _id(0);
uint64_t Instance::NewId()
{
    const uint64_t new_id = ++_id;
    return new_id;
}
uint64_t Instance::CurrentId()
{
    return _id;
}
}  // namespace IdGenerator
} // namespace chameleon

