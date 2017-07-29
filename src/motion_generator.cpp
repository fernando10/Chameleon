// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "motion_generator.h"
#include "glog/logging.h"
#include "math_utils.h"

namespace elninho
{

MotionGenerator::MotionGenerator(const MotionGeneratorOptions& options):
  options_(options){
}

RobotPoseVectorPtr MotionGenerator::GenerateMotion() const {
  RobotPoseVectorPtr robot_poses;

  switch (options_.motion_type) {
  case MotionTypes::Rectangle:
    robot_poses = GenerateRectangularMotion();
    break;
  case MotionTypes::Circle:
    robot_poses = GenerateCircularMotion();
    break;
  case MotionTypes::SineWave:
    break;
  case MotionTypes::StraightLine:
    robot_poses = GenerateStraightLine();
    break;
  default:
    LOG(ERROR) << " Motion type not supported. Unable to generate motion";
  }

  return robot_poses;
}

RobotPoseVectorPtr MotionGenerator::GenerateStraightLine() const {
  VLOG(1) << " Generating straight line motion.";
  RobotPoseVectorPtr robot_poses = std::make_shared<RobotPoseVector>();

  // start position
  robot_poses->push_back(RobotPose(0., Eigen::Vector2d(-kRectangleLength / 2,  0.)));
  Sophus::SE2d increment(0., Eigen::Vector2d(kRectangleLength/options_.num_steps, 0.));

  for (size_t ii = 1; ii < options_.num_steps; ++ii) {
    robot_poses->push_back(RobotPose(robot_poses->at(ii - 1).pose * increment));
  }

  return robot_poses;
}

RobotPoseVectorPtr MotionGenerator::GenerateRectangularMotion() const {
  VLOG(1) << " Generating rectangular motion.";

  RobotPoseVectorPtr robot_poses = std::make_shared<RobotPoseVector>();

  const double corner_offset = kRectangleCornerPercent * kRectangleLength;  // start cornering this distance [m] from the edge
  // assuming the length is the longest side (quarter turn)
  const double corner_perimiter = corner_offset * M_PI * 0.5;
  const double length_minus_corner = kRectangleLength - (2 * corner_offset);
  const double width_minus_corner = kRectangleWidth - (2 * corner_offset);

  const double total_perimiter =  2 * length_minus_corner
                                  + 2 * width_minus_corner
                                  + 4 * corner_perimiter;
  VLOG(3) << "total_perimiter: " << total_perimiter;

  const double dt = total_perimiter / options_.num_steps;
  VLOG(3) << "dt: " << dt;

  if (dt < 1e-16) {
    LOG(ERROR) << "dt too close to zero: " << dt;
    return robot_poses;
  }
  const size_t num_steps_length = std::floor(length_minus_corner / dt);
  const size_t num_steps_width = std::floor(width_minus_corner / dt);
  const size_t num_steps_corner = std::floor(corner_perimiter / dt);

  if (num_steps_corner == 0 || num_steps_length == 0 || num_steps_width == 0) {
    LOG(ERROR) << " Zero steps on a side or corner...shouldn't happen.";
    return robot_poses;
  }

  // segment velocities in the robot frame
  const double corner_omega = Deg2Rad(-90) / num_steps_corner;
  const double side_omega = 0;
  const Eigen::Vector2d corner_vel = Eigen::Vector2d(corner_offset / num_steps_corner, 0.);
  const Eigen::Vector2d length_vel = Eigen::Vector2d(length_minus_corner / num_steps_length, 0.);
  const Eigen::Vector2d width_vel = Eigen::Vector2d(width_minus_corner / num_steps_width, 0.);

  robot_poses->push_back(RobotPose(0., Eigen::Vector2d(-kRectangleLength / 2 + corner_offset, kRectangleWidth / 2)));

  // Top side of rectangle (start from one since the first pose has already been added) (length)
  for (size_t ii = 1; ii < num_steps_length; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(side_omega, length_vel));
  }

  // Top right corner
  for (size_t ii = 0; ii < num_steps_corner; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(corner_omega, corner_vel));
  }

  // Right side (width)
  for (size_t ii = 0; ii < num_steps_width; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(side_omega, width_vel));
  }

  // Bottom right corner
  for (size_t ii = 0; ii < num_steps_corner; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(corner_omega, corner_vel));
  }

  // Bottom side (length)
  for (size_t ii = 0; ii < num_steps_length; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(side_omega, length_vel));
  }

  // Bottom left corner
  for (size_t ii = 0; ii < num_steps_corner; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(corner_omega, corner_vel));
  }

  // Left side (width)
  for (size_t ii = 0; ii < num_steps_width; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(side_omega, width_vel));
  }

  // Top left corner
  for (size_t ii = 0; ii < num_steps_corner; ++ii) {
    robot_poses->push_back(robot_poses->back() * Sophus::SE2d(corner_omega, corner_vel));
  }

  return robot_poses;
}

RobotPoseVectorPtr MotionGenerator::GenerateCircularMotion() const {
  RobotPoseVectorPtr robot_poses = std::make_shared<RobotPoseVector>();
  return robot_poses;
}

RobotPoseVectorPtr MotionGenerator::GenerateSineWaveMotion() const {
  RobotPoseVectorPtr robot_poses = std::make_shared<RobotPoseVector>();
  return robot_poses;
}


MotionGenerator::RobotSpline MotionGenerator::GenerateSplineFromWaypoints(const RobotPoseVectorPtr &waypoints) const {
  VLOG(2) << " Generating spline from "  << waypoints->size() << " waypoints";

  Eigen::ArrayXXd positions(2, waypoints->size());
  Eigen::ArrayXd rotations(waypoints->size());

  for (size_t ii = 0; ii < waypoints->size(); ++ii) {
    positions.col(ii) = waypoints->at(ii).pose.translation();
    rotations(ii) = waypoints->at(ii).pose.so2().log();
  }

  RobotSpline result;
  // 2nd order spline to have smooth turns
  result.position_spline = Eigen::SplineFitting<Spline2d>::Interpolate(positions, 2);
  result.rotation_spline = Eigen::SplineFitting<Spline1d>::Interpolate(rotations, 2);

  return result;
}

RobotPoseVectorPtr MotionGenerator::GeneratePosesFromSpline(const RobotSpline &spline) const {
  VLOG(2) << " Generating poses from spline" ;

  RobotPoseVectorPtr out_poses = std::make_shared<RobotPoseVector>();
  double spline_position = 0;
  double spline_increment = 1.0 / options_.num_steps;

  // sample spline to get poses
  for (size_t ii = 0; ii < options_.num_steps; ++ii) {
    double theta = 0.;//spline.rotation_spline(spline_position)(0);
    PointType pt = spline.position_spline(spline_position);
    Eigen::Vector2d position(pt[0], pt[1]);

    out_poses->push_back(RobotPose(theta, position));
    spline_position += spline_increment;
  }
  return out_poses;
}

}
