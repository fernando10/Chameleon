// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/world_generator.h"
#include "glog/logging.h"
#include  "fmt/format.h"
#include "fmt/ostream.h"
#include "chameleon/id_provider.h"

namespace chameleon
{
LandmarkVectorPtr WorldGenerator::GetWorld() const {
  return map_;
}

LandmarkVectorPtr WorldGenerator::SampleWorld()  {

  if (noisy_map_ != nullptr) {
    // map has already been built, just return
    return noisy_map_;
  }

  noisy_map_ = std::make_shared<LandmarkVector>();
  if (map_ == nullptr || map_->empty()) {
    LOG(ERROR) << " Noisy map requested but no ground truth map built yet.";
    return noisy_map_;
  }

  for (const Landmark& lm : (*map_)) {
    Distribution dist(lm.vec(), Covariance2d(kMapSigma, kMapSigma, 0));
    MultivariateNormalVariable lm_var(dist);

    Landmark noisy_lm;
    Eigen::Vector2d sample_mean = lm_var();
    noisy_lm.SetPosition(sample_mean[0], sample_mean[1]);
    noisy_lm.covariance = dist.cov;
    noisy_lm.id = lm.id;
    noisy_lm.active = false;
    noisy_map_->push_back(noisy_lm);
  }
  return noisy_map_;
}

LandmarkVectorPtr WorldGenerator::GenerateWorld(const RobotPoseVectorPtr& robot_poses, WorldTypes type) {
  LandmarkVectorPtr map = std::make_shared<LandmarkVector>();
  VLOG(1) << " Generating landmarks for path.";

  if(robot_poses == nullptr || robot_poses->size() == 0){
    LOG(ERROR) << "Map requested but no robot poses passed in.";
    return map;
  }

  const size_t lm_per_m = 2;

  Eigen::Vector2d x_range = Eigen::Vector2d::Zero();
  Eigen::Vector2d y_range = Eigen::Vector2d::Zero();
  Eigen::Vector2d start = robot_poses->at(0).translation();

  for (const RobotPose& p : *robot_poses) {
    if (p.x() < x_range[0]) {
      x_range[0] = p.x();
    }
    else if (p.x() > x_range[1]) {
      x_range[1] = p.x();
    }

    if (p.y() < y_range[0]) {
      y_range[0] = p.y();
    }
    else if (p.y() > y_range[1]) {
      y_range[1] = p.y();
    }
  }

  if (type == WorldTypes::MimicTrajctory) {


    Eigen::Vector2d scale(0.3, 0.5);  // x and y scale

    Eigen::Vector2d outer_offset(- ((1+ scale[0]) * (x_range[1]) - x_range[1])/2,
        -((1 + scale[1]) * (y_range[0]) - y_range[0])/2);

    Eigen::Vector2d inner_offset(- ((1- scale[0]) * (x_range[1]) - x_range[1])/2,
        -((1 - scale[1]) * (y_range[0]) - y_range[0])/2);

    for(size_t i = 0; i < robot_poses->size(); ++i) {
      if (i % 7 == 0) { // add a lm every 10th pose
        Eigen::Vector2d trans = robot_poses->at(i).translation();
        trans[0] *= (scale[0] + 1);
        trans[1] *= (scale[1] + 1);
        trans += outer_offset;
        map->push_back(Landmark(IdGenerator::Instance::NewId(), trans[0] ,
                       trans[1]));
      }
    }

    for(size_t i = 0; i < robot_poses->size(); ++i) {
      if (i % 7 == 0) { // add a lm every 13th pose
        Eigen::Vector2d trans = robot_poses->at(i).translation();
        trans[0] *= (1 - scale[0]);
        trans[1] *= (1 - scale[1]);
        trans += inner_offset;
        map->push_back(Landmark(IdGenerator::Instance::NewId(), trans[0] ,
                       trans[1]));
      }
    }

  }

  if (type == WorldTypes::RectangularField) {
    //Eigen::Vector2d max_translation = Eigen::Vector2d::Zero();

    double length = std::abs(x_range[1] - x_range[0]) * 1.2;
    double width = std::abs(y_range[1] - y_range[0]) * 1.2;


    // curbs
    for (size_t lm_idx = 0; lm_idx < lm_per_m *length; ++lm_idx) {
      map->push_back(Landmark(IdGenerator::Instance::NewId(), 1./lm_per_m * lm_idx + start[0] - 10 ,
                     start[1] + 3));
    }

    for (size_t lm_idx = 0; lm_idx < lm_per_m * length; ++lm_idx) {
      map->push_back(Landmark(IdGenerator::Instance::NewId(), 1./lm_per_m * lm_idx + start[0] - 10 ,
                     start[1] - width));
    }
  }



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

void WorldGenerator::ChangeLandmarks(std::vector<uint64_t> lm_ids) {
  if (map_ == nullptr) {
    LOG(ERROR) << "Tried to change landmarks but map does not exist.";
    return;
  }

  VLOG(1) << " changing landmarks (" << lm_ids.size() << " )";

  // constant change for now...just add 1 to the x direction
  //Eigen::Vector2d delta(1, 0);
  for (const uint64_t lm_id : lm_ids) {
    for (LandmarkVector::iterator it = map_->begin(); it != std::end(*map_); ++it) {
      if (it->id == lm_id) {
        VLOG(1) << "Found landmark with id: " << lm_id << ", changing.";
        it->SetPosition(it->x() + 1, it->y());
        break;
      }
    }
  }
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
      } else {
        ++it;
      }
    }
  }
  return res;
}

} // namespace chameleon
