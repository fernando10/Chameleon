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

RobotPoseVector MotionGenerator::GenerateMotion(){
  RobotPoseVector robot_poses;

  switch (options_.motion_type) {
  case MotionTypes::Rectangle:
    robot_poses = GenerateRectangularMotion();
    break;
  case MotionTypes::Circle:
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

RobotPoseVector MotionGenerator::GenerateStraightLine() {
  VLOG(1) << " Generating straight line motion.";
  RobotPoseVector robot_poses;

  // start position
  robot_poses.push_back(RobotPose(0., Eigen::Vector2d(-kRectangleLength / 2,  0.)));
  Sophus::SE2d increment(0., Eigen::Vector2d(kRectangleLength/options_.num_steps, 0.));

  for (size_t ii = 1; ii < options_.num_steps; ++ii) {
    robot_poses.push_back(RobotPose(robot_poses.at(ii - 1).pose * increment));
  }

  return robot_poses;
}

RobotPoseVector MotionGenerator::GenerateRectangularMotion() {
  VLOG(1) << " Generating rectangular motion.";

  RobotPoseVector waypoints;
  const double corner_offset = 1;  // start cornering this distance [m] from the edge

  const double half_length = kRectangleLength / 2;
  const double half_width = kRectangleWidth / 2;

  // we will add the waypoints for the corners of the rectangle, which will be traversed clockwise
  // from a top-down view, x axis is positive to the right, and y axis is positive going up
  // top left corner
  waypoints.push_back(RobotPose(Deg2Rad(90), Eigen::Vector2d(-half_length, half_width - corner_offset)));
  waypoints.push_back(RobotPose(Deg2Rad(45), Eigen::Vector2d(-half_length, half_width)));
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(-half_length + corner_offset, half_width)));

  // top middle
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(0, half_width)));

  // top right corner
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(half_length - corner_offset, half_width)));
  waypoints.push_back(RobotPose(Deg2Rad(-45), Eigen::Vector2d(half_length, half_width)));
  waypoints.push_back(RobotPose(Deg2Rad(-90), Eigen::Vector2d(half_length, half_width - corner_offset)));

  // right middle
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(half_length, 0)));

  // bottom right corner
  waypoints.push_back(RobotPose(Deg2Rad(-90), Eigen::Vector2d(half_length , -half_width + corner_offset)));
  waypoints.push_back(RobotPose(Deg2Rad(-135), Eigen::Vector2d(half_length, -half_width)));
  waypoints.push_back(RobotPose(Deg2Rad(-180), Eigen::Vector2d(half_length - corner_offset, -half_width)));

  // bottom middle
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(0, -half_width)));

  // bottom left corner
  waypoints.push_back(RobotPose(Deg2Rad(-180), Eigen::Vector2d(-half_length + corner_offset , -half_width)));
  waypoints.push_back(RobotPose(Deg2Rad(135), Eigen::Vector2d(-half_length, -half_width)));
  waypoints.push_back(RobotPose(Deg2Rad(90), Eigen::Vector2d(-half_length, -half_width + corner_offset)));

  // left middle
  waypoints.push_back(RobotPose(0., Eigen::Vector2d(-half_length, 0)));

  // closing the loop
  waypoints.push_back(RobotPose(Deg2Rad(90), Eigen::Vector2d(-half_length, half_width - corner_offset)));


  VLOG(2) << "Generated "  << waypoints.size() << " waypoints.";
  // run a spline through the waypoints
  RobotSpline spline = GenerateSplineFromWaypoints(waypoints);

  // and generate poses sampled from the spline
  return GeneratePosesFromSpline(spline);
}

MotionGenerator::RobotSpline MotionGenerator::GenerateSplineFromWaypoints(const RobotPoseVector &waypoints) {
  VLOG(2) << " Generating spline from "  << waypoints.size() << " waypoints";

  Eigen::ArrayXXd positions(2, waypoints.size());
  Eigen::ArrayXd rotations(waypoints.size());

  for (size_t ii = 0; ii < waypoints.size(); ++ii) {
    positions.col(ii) = waypoints.at(ii).pose.translation();
    rotations(ii) = waypoints.at(ii).pose.so2().log();
  }

  RobotSpline result;
  // 2nd order spline to have smooth turns
  result.position_spline = Eigen::SplineFitting<Spline2d>::Interpolate(positions, 2);
  result.rotation_spline = Eigen::SplineFitting<Spline1d>::Interpolate(rotations, 2);

  return result;
}

RobotPoseVector MotionGenerator::GeneratePosesFromSpline(const RobotSpline &spline) {
  VLOG(2) << " Generating poses from spline" ;

  RobotPoseVector out_poses;
  double spline_position = 0;
  double spline_increment = 1.0 / options_.num_steps;

  // sample spline to get poses
  for (size_t ii = 0; ii < options_.num_steps; ++ii) {
    double theta = 0.;//spline.rotation_spline(spline_position)(0);
    PointType pt = spline.position_spline(spline_position);
    Eigen::Vector2d position(pt[0], pt[1]);

    out_poses.push_back(RobotPose(theta, position));
    spline_position += spline_increment;
  }
  return out_poses;
}

}
