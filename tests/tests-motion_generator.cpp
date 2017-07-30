// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "summersimulator/motion_generator.h"
#include "summersimulator/path_generator.h"
#include "summersimulator/data_generator.h"

using namespace summer;

TEST_CASE("Noise-free odometry integrates back to original path") {

  DataGenerator::DataGeneratorOptions options; // use default options

  std::unique_ptr<PathGenerator> path_generator = util::make_unique<PathGenerator>(options.path_options);
  std::unique_ptr<MotionGenerator> motion_generator = util::make_unique<MotionGenerator>();

  SECTION( "rectangular path" ) {
    options.path_options.motion_type = PathGenerator::PathTypes::Rectangle;

    // generate a ground truth path (no noise in these poses)
    RobotPoseVectorPtr ground_truth_robot_path = path_generator->GeneratePath();

    // load the path into the motion (odometry) generator
    motion_generator->SetPath(ground_truth_robot_path);

    RobotPoseVectorPtr integrated_robot_path = std::make_shared<RobotPoseVector>();
    integrated_robot_path->push_back(ground_truth_robot_path->at(0)); // same initial condition

    for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {

      OdometryMeasurement noise_free_odometry =
          motion_generator->GenerateNoiseFreeOdometryMeasurement(ii);

      // now propagate the odometry
      RobotPose integrated_pose = motion_generator->PropagateMeasurement(noise_free_odometry,
                                                                         integrated_robot_path->at(ii));
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

    // generate a ground truth path (no noise in these poses)
    RobotPoseVectorPtr ground_truth_robot_path = path_generator->GeneratePath();

    // load the path into the motion (odometry) generator
    motion_generator->SetPath(ground_truth_robot_path);

    RobotPoseVectorPtr integrated_robot_path = std::make_shared<RobotPoseVector>();
    integrated_robot_path->push_back(ground_truth_robot_path->at(0)); // same initial condition

    for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {

      OdometryMeasurement noise_free_odometry =
          motion_generator->GenerateNoiseFreeOdometryMeasurement(ii);

      // now propagate the odometry
      RobotPose integrated_pose = motion_generator->PropagateMeasurement(noise_free_odometry,
                                                                         integrated_robot_path->at(ii));
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

  SECTION( "straight line path" ) {
    options.path_options.motion_type = PathGenerator::PathTypes::StraightLine;

    // generate a ground truth path (no noise in these poses)
    RobotPoseVectorPtr ground_truth_robot_path = path_generator->GeneratePath();

    // load the path into the motion (odometry) generator
    motion_generator->SetPath(ground_truth_robot_path);

    RobotPoseVectorPtr integrated_robot_path = std::make_shared<RobotPoseVector>();
    integrated_robot_path->push_back(ground_truth_robot_path->at(0)); // same initial condition

    for (size_t ii = 0; ii < ground_truth_robot_path->size() - 1; ++ii) {

      OdometryMeasurement noise_free_odometry =
          motion_generator->GenerateNoiseFreeOdometryMeasurement(ii);

      // now propagate the odometry
      RobotPose integrated_pose = motion_generator->PropagateMeasurement(noise_free_odometry,
                                                                         integrated_robot_path->at(ii));
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
