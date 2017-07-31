// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/odometry_generator.h"
#include "glog/logging.h"

namespace chameleon
{

OdometryGenerator::OdometryGenerator(const RobotPoseVectorPtr robot_poses): robot_poses_(robot_poses){
}

OdometryGenerator::OdometryGenerator(const Eigen::Vector4d odometry_noise): odometry_noise_(odometry_noise) {
}

void OdometryGenerator::SetPath(const RobotPoseVectorPtr robot_poses) {
  robot_poses_ = robot_poses;
}

RobotPoseVectorPtr OdometryGenerator::GetPath() {
  return robot_poses_;
}

OdometryMeasurement OdometryGenerator::GenerateNoiseFreeOdometryMeasurement(size_t step) const {
  if (robot_poses_ == nullptr) {
    LOG(ERROR) << "Path not initalized but generate odometry was called...";
    return OdometryMeasurement();
  }

  if (step >= robot_poses_->size()) {
    LOG(ERROR) << " inex out of bounds, only " << robot_poses_->size() << " poses are here.";
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

OdometryMeasurement OdometryGenerator::GenerateNoisyOdometryMeasurement(size_t step) const {
  OdometryMeasurement noise_free_odometry = GenerateNoiseFreeOdometryMeasurement(step);
  const double theta_1 = noise_free_odometry.theta_1;
  const double theta_2 = noise_free_odometry.theta_2;
  const double trans = noise_free_odometry.translation;

  Eigen::Matrix<double, 1, 1> variance;
  Eigen::Matrix<double, 1, 1> mean;
  variance(0) = odometry_noise_[0] * Square(theta_1) + odometry_noise_[1] * Square(trans);
  mean(0) = theta_1;
  MultivariateNormalVariable theta_1_sample(variance, mean);

  variance(0) = odometry_noise_[2] * Square(trans) + odometry_noise_[3] * (Square(theta_1) + Square(theta_2));
  mean(0) = trans;
  MultivariateNormalVariable translation_sample(variance, mean);

  variance(0) = odometry_noise_[0] * Square(theta_2) + odometry_noise_[1] * Square(trans);
  mean(0) = theta_2;
  MultivariateNormalVariable theta_2_sample(variance, mean);

  // sample from the normal distribution to obtain the noisy values
  double noisy_theta_1 = theta_1_sample()[0];
  double noisy_trans = translation_sample()[0];
  double noisy_theta_2 = theta_2_sample()[0];

//  VLOG(3) << "orig values: [" << theta_1 << ", " << trans << ", " << theta_2;
//  VLOG(3) << "noisy values: [" << noisy_theta_1 << ", " << noisy_trans << ", " << noisy_theta_2;
//  VLOG(3) << "A values: [" << theta_1_sample.A << ", " << translation_sample.A << ", " << theta_2_sample.A;

  return OdometryMeasurement(noisy_theta_1, noisy_trans, noisy_theta_2);
}

OdometryMeasurementVectorPtr OdometryGenerator::GenereteOdometry(bool noisy) const {
  OdometryMeasurementVectorPtr odometry_measurements = std::make_shared<OdometryMeasurementVector>();

  for (size_t ii = 0; ii < robot_poses_->size() - 1; ++ii) {
    OdometryMeasurement odometry;

    if (noisy) {
      odometry = GenerateNoisyOdometryMeasurement(ii);
    }
    else {
      odometry = GenerateNoiseFreeOdometryMeasurement(ii);
    }
    odometry_measurements->push_back(odometry);
  }

  return odometry_measurements;
}


RobotPose OdometryGenerator::PropagateMeasurement(const OdometryMeasurement &meas, const RobotPose &start_pose) const {
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

}  // namespace chameleon
