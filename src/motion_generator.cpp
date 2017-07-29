// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "summersimulator/motion_generator.h"
#include "glog/logging.h"

namespace summer
{

MotionGenerator::MotionGenerator(const RobotPoseVectorPtr robot_poses): robot_poses_(robot_poses){
}

void MotionGenerator::SetPath(const RobotPoseVectorPtr robot_poses) {
  robot_poses_ = robot_poses;
}

RobotPoseVectorPtr MotionGenerator::GetPath() {
  return robot_poses_;
}

OdometryMeasurement MotionGenerator::GenerateOdometryMeasurement(size_t step) {
  if (robot_poses_ == nullptr) {
    LOG(ERROR) << "Path not initalized but generate odometry was called...";
    return OdometryMeasurement();
  }

  if (step == robot_poses_->size() - 1) {
    LOG(ERROR) << "Cannot generate odometry for last measurement, nothing to differentiate";
    return OdometryMeasurement();
  }

  RobotPose& prev_pose = robot_poses_->at(step);
  RobotPose& current_pose = robot_poses_->at(step + 1);

  double theta_1 = 0;
  double theta_2 = 0;
  double translation;

  theta_1 = std::atan2(current_pose.y() - prev_pose.y(), current_pose.x() - prev_pose.x()) - prev_pose.theta();
  translation = (current_pose.translation() - prev_pose.translation()).norm();
  theta_2 = current_pose.theta() - prev_pose.theta() - theta_1;

  return OdometryMeasurement(theta_1, translation, theta_2);
}

RobotPose MotionGenerator::PropagateMeasurement(const OdometryMeasurement &meas, const RobotPose &start_pose) {
  RobotPose propagated_pose = start_pose;

  // first rotation
  propagated_pose.SetTheta(propagated_pose.theta() + meas.theta_1);
  // translation
  propagated_pose.x() += meas.translation * std::cos(propagated_pose.theta());
  propagated_pose.y() += meas.translation * std::sin(propagated_pose.theta());
  // second rotation
  propagated_pose.SetTheta(propagated_pose.theta() + meas.theta_2);

  return propagated_pose;
}

}  // namespace elninho
