// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/world_generator.h"
#include "glog/logging.h"
#include  "fmt/format.h"
#include "fmt/ostream.h"
#include "chameleon/id_provider.h"

namespace chameleon
{
//LandmarkVectorPtr WorldGenerator::GenerateWorld(const RobotPoseVectorPtr& robot_poses) {
//  LandmarkVectorPtr map = std::make_shared<LandmarkVector>();
//  VLOG(1) << " Generating landmarks for path.";

//  if(robot_poses == nullptr || robot_poses->size() == 0){
//    LOG(ERROR) << "Map requested but no robot poses passed in.";
//    return map;
//  }

//  double path_length = GetTotalDistanceTraveled(robot_poses);

//  if(path_length > 0) {
//    // TODO: improve algorithm for distrubuting landmarks
//    size_t num_landmarks = std::floor(path_length * kLandmarkDensity);
//    size_t num_landmarks_per_pose = std::ceil((double)num_landmarks / (double)robot_poses->size());

//    if (num_landmarks_per_pose > 2) { num_landmarks_per_pose = 2; }

//    for(size_t pose_idx = 1; pose_idx < robot_poses->size(); ++pose_idx) {
//      for(size_t lm_idx = 0; lm_idx < num_landmarks_per_pose; ++lm_idx) {
//        Landmark lm_r(0, lm_idx % 2 == 0 ? kLandmarkdDistance : -kLandmarkdDistance);  // landmark in robot frame
//        map->push_back(Landmark(robot_poses->at(pose_idx).pose.so2().matrix() * lm_r.vec()));  // transfer to world frame and add to map
//      }
//    }

//    //VLOG(1) << fmt::format("Generated {} landmarks.", map->size());


//  } else {
//    LOG(ERROR) << "Distance traveled <= 0, should not happen...";
//  }

//  return map;
//}

LandmarkVectorPtr WorldGenerator::GetWorld() const {
  return map_;
}

LandmarkVectorPtr WorldGenerator::GenerateWorld(const RobotPoseVectorPtr& robot_poses) {
  LandmarkVectorPtr map = std::make_shared<LandmarkVector>();
  VLOG(1) << " Generating landmarks for path.";

  if(robot_poses == nullptr || robot_poses->size() == 0){
    LOG(ERROR) << "Map requested but no robot poses passed in.";
    return map;
  }

  Eigen::Vector2d max_translation = Eigen::Vector2d::Zero();
  for (const auto& p : *robot_poses) {
    if( std::abs(p.pose.translation().y()) > max_translation.y()) {
      max_translation.y() = std::abs(p.pose.translation().y());
    }

    if( std::abs(p.pose.translation().x()) > max_translation.x()) {
      max_translation.x() = std::abs(p.pose.translation().x());
    }
  }

  for (size_t lm_idx = 0; lm_idx < 8; ++lm_idx) {
    map->push_back(Landmark(IdGenerator::Instance::NewId(), 20/7. * lm_idx - 4,
                            max_translation.y() * 0.5 + 1 * lm_idx % 2));
  }

  for (size_t lm_idx = 0; lm_idx < 8; ++lm_idx) {
    map->push_back(Landmark(IdGenerator::Instance::NewId(), 20/7. * lm_idx - 4,
                            -max_translation.y() * 1.5 - 1 * lm_idx % 2));
  }

  double mid_pt =  (max_translation.y() * 0.5 - max_translation.y() * 1.5 ) /2.;
  map->push_back(Landmark(IdGenerator::Instance::NewId(),-4.5, mid_pt));
  map->push_back(Landmark(IdGenerator::Instance::NewId(), 20 - 3.5 , mid_pt));


  map_ = map; // store the map locally in case it's neeeded in the future
  return map;
}

double WorldGenerator::GetTotalDistanceTraveled(const RobotPoseVectorPtr& poses) {
  double distance_traveled = 0;

  if(poses->size() > 1) {
    for (size_t ii = 1; ii < poses->size(); ++ii) {
      distance_traveled += (poses->at(ii-1).pose.inverse() * poses->at(ii).pose).translation().norm();
    }
  }
  return distance_traveled;
}

bool WorldGenerator::RemoveLandmarks(std::vector<uint64_t> lm_ids) {
  if (map_ == nullptr) {
    LOG(ERROR) << "Tried to remove landmark but map does not exist.";
    return false;
  }

  bool res = false;

  for (const uint64_t lm_id : lm_ids) {
    res = false;
    for (LandmarkVector::iterator it = map_->begin(); it != std::end(*map_);) {
      if (it->id == lm_id) {
        VLOG(1) << "Found landmark with id: " << lm_id << ", removing from map.";
        map_->erase(it);
        res = true;
        break;
      }
    }
  }

  return res;
}

} // namespace chameleon
