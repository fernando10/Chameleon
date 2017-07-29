// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "data_generator.h"
#include "util.h"

namespace elninho
{

DataGenerator::DataGenerator(const DataGeneratorOptions &options):
  options_(options), world_generator_(util::make_unique<WorldGenerator>()),
  motion_generator_(util::make_unique<MotionGenerator>()){
}

DataGenerator::GenerateSimulatedData() {

}

} // namespace elninho
