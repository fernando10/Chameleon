// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "motion_generator.h"
#include "glog/logging.h"
namespace elninho
{

MotionGenerator::MotionGenerator(const MotionGeneratorOptions& options):
  options_(options){
}

RobotPoseVector MotionGenerator::GenerateMotion(){
  RobotPoseVector robot_poses;

  if (options_.delta_t > 1.0) {
    LOG(ERROR) << "Delta t should be < 1.0 for motion generation.";
    return robot_poses;
  }



//  int n_steps = 1 / options_.delta_t;
//  int index = (n_steps % (std::floor(1/options_.delta_t) * 5));

//  if (index == 0) {
//    m = [0; options_.delta_t*100; 0];
//  }
//  else if (index == 1*floor(1/options_.delta_t)) {
//    m = [0; options_.delta_t*100; 0];
//  }
//  else if (index == 2*floor(1/options_.delta_t)) {
//    m = [Deg2Rad(45); deltaT*100; Deg2Rad(45)];
//  }
//  else if (index == 3*floor(1/options_.delta_t)) {
//    m = [0; deltaT*100; 0];
//  }
//  else if (index == 4*floor(1/options_.delta_t) {
//    m = [Deg2Rad(45); 0; Deg2Rad(45)];
//  }
//  else {
//    m = [0; deltaT*100; 0];
//}

  return robot_poses;

}

}
