// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/data_generator.h"
#include "chameleon/util.h"
#include "glog/logging.h"

namespace chameleon
{

DataGenerator::DataGenerator(const DataGeneratorOptions& options):
  options_(options),
  world_generator_(util::make_unique<WorldGenerator>()),
  path_generator_(util::make_unique<PathGenerator>(options.path_options)),
  odometry_generator_(util::make_unique<OdometryGenerator>(options.odometry_noise)) {
}

bool DataGenerator::GenerateSimulatedData(SimData* data) {

  // generate a ground truth path (no noise in these poses)
  RobotPoseVectorPtr ground_truth_robot_path = path_generator_->GeneratePath();

  if (ground_truth_robot_path->empty()) {
    LOG(ERROR) << "No poses generated.";
    return false;
  }

  if (options_.generate_landmarks) {
     LandmarkVectorPtr ground_truth_landmarks =  world_generator_->GenerateWorld(ground_truth_robot_path);
     data->debug.ground_truth_map = ground_truth_landmarks;
  }

  RobotPose noisy_robot_pose = ground_truth_robot_path->at(0);
  RobotPose noise_free_robot_pose = ground_truth_robot_path->at(0);

  data->debug.ground_truth_poses->push_back(noise_free_robot_pose);
  data->debug.noisy_poses->push_back(noisy_robot_pose);

  // load the path into the motion (odometry) generator
  odometry_generator_->SetPath(ground_truth_robot_path);

  for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {
    OdometryMeasurement noise_free_odometry =
        odometry_generator_->GenerateNoiseFreeOdometryMeasurement(ii);
    RobotPose new_noise_free_pose =
        odometry_generator_->PropagateMeasurement(noise_free_odometry, noise_free_robot_pose);
    noise_free_robot_pose = new_noise_free_pose;

    OdometryMeasurement noisy_odometry =
        odometry_generator_->GenerateNoisyOdometryMeasurement(ii);
    RobotPose new_noisy_pose =
        odometry_generator_->PropagateMeasurement(noisy_odometry, noisy_robot_pose);
    noisy_robot_pose = new_noisy_pose;

    data->times.push_back(ii);
    data->debug.ground_truth_poses->push_back(new_noise_free_pose);
    data->debug.noisy_poses->push_back(new_noisy_pose);
    data->debug.noise_free_odometry->push_back(noise_free_odometry);
    data->debug.noisy_odometry->push_back(noisy_odometry);
  }

  return true;
}
} // namespace chameleon
