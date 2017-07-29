// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "types.h"
#include "util.h"

namespace elninho
{

struct MotionGeneratorOptions {
  double delta_t = 0;
};

class MotionGenerator {
 public:

  MotionGenerator(const MotionGeneratorOptions& options);

  RobotPoseVector GenerateMotion();

 private:
  const MotionGeneratorOptions& options_;

};

}
