// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "summersimulator/data_generator.h"
#include "util.h"
#include "glog/logging.h"

namespace summer
{

DataGenerator::DataGenerator(const DataGeneratorOptions& options):
  options_(options),
  world_generator_(util::make_unique<WorldGenerator>()),
  path_generator_(util::make_unique<PathGenerator>(options.path_options)),
  motion_generator_(util::make_unique<MotionGenerator>(options.odometry_noise)) {
}

bool DataGenerator::GenerateSimulatedData(SimData* data) {

  // generate a ground truth path (no noise in these poses)
  RobotPoseVectorPtr ground_truth_robot_path = path_generator_->GeneratePath();

  if (ground_truth_robot_path->empty()) {
    LOG(ERROR) << "No poses generated.";
    return false;
  }

  RobotPoseVectorPtr noisy_robot_path = std::make_shared<RobotPoseVector>();
  noisy_robot_path->push_back(ground_truth_robot_path->at(0));  // set start pose

  // load the path into the motion (odometry) generator
  motion_generator_->SetPath(ground_truth_robot_path);

  for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {

    OdometryMeasurement noisy_odometry =
        motion_generator_->GenerateNoisyOdometryMeasurement(ii);

    RobotPose new_noisy_pose =
        motion_generator_->PropagateMeasurement(noisy_odometry, noisy_robot_path->back());
    noisy_robot_path->push_back(new_noisy_pose);

  }

  data->debug.ground_truth_poses = ground_truth_robot_path;
  data->debug.noisy_poses = noisy_robot_path;

  return true;
}

} // namespace summer
