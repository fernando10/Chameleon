// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "world_generator.h"
#include "glog/logging.h"

namespace elninho
{
WorldGenerator::GenerateWorld(WorldTypes type) {
  switch(type) {
    case WorldTypes::RectangularField:
      VLOG(1) << "Generating rectangular field world";
    break;
  default:
    LOG(ERROR) << "World type not supported yet.";
  }

}

void WorldGenerator::GenerateRectangularField() {
  int num_landmarks_length = std::floor(kLandmarkDensity * kFieldLength);
}



} // namespace elninho
