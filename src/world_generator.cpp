// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "summersimulator/world_generator.h"
#include "glog/logging.h"
#include  "fmt/format.h"

namespace summer
{
LandmarkVectorPtr WorldGenerator::GenerateWorld(const RobotPoseVectorPtr& robot_poses) {
  LandmarkVectorPtr map = std::make_shared<LandmarkVector>();
  VLOG(1) << " Generating landmarks for path.";

  if(robot_poses == nullptr || robot_poses->size() == 0){
    LOG(ERROR) << "Map requested but no robot poses passed in.";
    return map;
  }

  double path_length = GetTotalDistanceTraveled(robot_poses);

  if(path_length > 0) {
    // TODO: improve algorithm for distrubuting landmarks
    size_t num_landmarks = std::floor(path_length * kLandmarkDensity);
    size_t num_landmarks_per_pose = std::ceil((double)num_landmarks / (double)robot_poses->size());

    if (num_landmarks_per_pose > 2) { num_landmarks_per_pose = 2; }

    for(size_t pose_idx = 1; pose_idx < robot_poses->size(); ++pose_idx) {
      for(size_t lm_idx = 0; lm_idx < num_landmarks_per_pose; ++lm_idx) {
        Landmark lm_r(0, lm_idx % 2 == 0 ? kLandmarkdDistance : -kLandmarkdDistance);  // landmark in robot frame
        map->push_back(Landmark(robot_poses->at(pose_idx).pose.so2().matrix() * lm_r.vec()));  // transfer to world frame and add to map
      }
    }

    //VLOG(1) << fmt::format("Generated {} landmarks.", map->size());


  } else {
    LOG(ERROR) << "Distance traveled <= 0, should not happen...";
  }

  return map;
}

double WorldGenerator::GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses) {
  double distance_traveled = 0;

  if(poses->size() > 1) {
    for (size_t ii = 1; ii < poses->size(); ++ii) {
      distance_traveled += (poses->at(ii-1).pose.inverse() * poses->at(ii).pose).translation().norm();
    }
  }
  //VLOG(1) << fmt::format("Path of {} poses, total distance traveled: {}m", poses->size(), distance_traveled);
  return distance_traveled;
}



} // namespace elninho
