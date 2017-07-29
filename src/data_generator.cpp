// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "summersimulator/data_generator.h"
#include "util.h"

namespace summer
{

DataGenerator::DataGenerator(const DataGeneratorOptions& options):
  options_(options),
  world_generator_(util::make_unique<WorldGenerator>()),
  path_generator_(util::make_unique<PathGenerator>(options.path_options)),
  motion_generator_(util::make_unique<MotionGenerator>()) {
}

DataGenerator::GenerateSimulatedData() {

  // generate a ground truth path (no noise in these poses)
  RobotPoseVectorPtr ground_truth_robot_path = path_generator_->GeneratePath();

  // load the path into the motion (odometry) generator
  motion_generator_->SetPath(ground_truth_robot_path);

  // generate the noise-free odometry measurements from the ground truth poses
  // then add noise and integrate to generate the full noisy or 'real' robot
  // trajectory
  for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {

    OdometryMeasurement noise_free_odometry =
        motion_generator_->GenerateOdometryMeasurement(ii);

  }




}

} // namespace elninho
