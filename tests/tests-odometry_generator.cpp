// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "chameleon/odometry_generator.h"
#include "chameleon/path_generator.h"
#include "chameleon/data_generator.h"

using namespace chameleon;

TEST_CASE("Noise-free odometry integrates back to original path") {

  DataGenerator::DataGeneratorOptions options; // use default options

  std::unique_ptr<PathGenerator> path_generator = util::make_unique<PathGenerator>(options.path_options);
  std::unique_ptr<OdometryGenerator> odometry_generator = util::make_unique<OdometryGenerator>(options.odometry_noise);

  SECTION( "rectangular path" ) {
    options.path_options.motion_type = PathGenerator::PathTypes::Rectangle;

    // generate a ground truth path (no noise in these poses)
    RobotPoseVectorPtr ground_truth_robot_path = path_generator->GetRobotPath();

    // load the path into the motion (odometry) generator
    odometry_generator->SetPath(ground_truth_robot_path);

    RobotPoseVectorPtr integrated_robot_path = std::make_shared<RobotPoseVector>();

    for (size_t ii = 0; ii < ground_truth_robot_path->size(); ++ii) {

      OdometryMeasurement noise_free_odometry =
          odometry_generator->GenerateNoiseFreeOdometryMeasurement(ii);

      // now propagate the odometry
      RobotPose integrated_pose = odometry_generator->PropagateMeasurement(noise_free_odometry);
      integrated_robot_path->push_back(integrated_pose);
    }

    // check same number of elements
    REQUIRE( integrated_robot_path->size() == ground_truth_robot_path->size() );


    // check same end position
    REQUIRE( integrated_robot_path->back().translation().norm() ==
              Approx(ground_truth_robot_path->back().translation().norm()) );
    REQUIRE( integrated_robot_path->back().theta() ==
              Approx(ground_truth_robot_path->back().theta()) );

  }

  SECTION( "circular path" ) {
    options.path_options.motion_type = PathGenerator::PathTypes::Circle;
    path_generator->RebuildPath();

    // generate a ground truth path (no noise in these poses)
    RobotPoseVectorPtr ground_truth_robot_path = path_generator->GetRobotPath();

    // load the path into the motion (odometry) generator
    odometry_generator->SetPath(ground_truth_robot_path);

    RobotPoseVectorPtr integrated_robot_path = std::make_shared<RobotPoseVector>();

    for (size_t ii = 0; ii < ground_truth_robot_path->size(); ++ii) {

      OdometryMeasurement noise_free_odometry =
          odometry_generator->GenerateNoiseFreeOdometryMeasurement(ii);

      // now propagate the odometry
      RobotPose integrated_pose = odometry_generator->PropagateMeasurement(noise_free_odometry);
      integrated_robot_path->push_back(integrated_pose);
    }

    // check same number of elements
    REQUIRE( integrated_robot_path->size() == ground_truth_robot_path->size() );

    // check same end position
    REQUIRE( integrated_robot_path->back().translation().norm() ==
              Approx(ground_truth_robot_path->back().translation().norm()) );
    REQUIRE( integrated_robot_path->back().theta() ==
              Approx(ground_truth_robot_path->back().theta()) );
  }
}
