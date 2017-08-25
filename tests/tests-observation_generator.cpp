// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "chameleon/path_generator.h"
#include "chameleon/world_generator.h"
#include "chameleon/observation_generator.h"
#include "chameleon/data_generator.h"

using namespace chameleon;

TEST_CASE("Only visible landmarks should generate observations") {
  RobotPoseVectorPtr poses = std::make_shared<RobotPoseVector>();
  LandmarkVectorPtr map = std::make_shared<LandmarkVector>();
  ObservationGenerator observation_generator(map, poses);

  poses->push_back(RobotPose()); // add a single pose at the origin, lookng down the positive x axis

  SECTION("visible landmark within range") {
    double range = poses->back().range;
    map->push_back(Landmark(range, 0));

    RangeFinderObservationVector obs = observation_generator.GenerateObservations(0);

    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 0.);
    REQUIRE( obs.at(0).observation.theta == 0. );
    REQUIRE( obs.at(0).observation.range == range );
  }

  SECTION("visible landmark out of range") {
    double range = poses->back().range;
    map->push_back(Landmark(range + 0.1, 0));

    RangeFinderObservationVector obs = observation_generator.GenerateObservations(0);

    REQUIRE( obs.size() == 0 );
  }

  SECTION("landmark within range but ouside field of view") {
    double range = poses->back().range;
    double fov = poses->back().field_of_view;
    double angle = fov / 2 + 0.4;  // add a bit to make the landmark out of range

    map->push_back(Landmark(range * std::cos(angle), range * std::sin(angle)));

    RangeFinderObservationVector obs = observation_generator.GenerateObservations(0);

    REQUIRE( obs.size() == 0 );
  }

  SECTION("landmark within range and feild of view but behind robot") {
    double range = poses->back().range;

    map->push_back(Landmark(-range, 0));

    observation_generator.Reset(map, poses);
    RangeFinderObservationVector obs = observation_generator.GenerateObservations(0);

    REQUIRE( obs.size() == 0 );
  }

  SECTION("robot not at origin - transfer landmark to robot") {
    poses->push_back(RobotPose(M_PI/4, 3., 3.)); // create robot at (3,3) with 45deg. orientation
    map->push_back(Landmark(2, 2));
    RangeFinderObservationVector obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 0 );

    map->push_back(Landmark(4, 4));
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 1);
    REQUIRE( obs.at(0).observation.theta == 0 );
    REQUIRE( obs.at(0).observation.range == Approx(1/std::cos(M_PI/4)));

    // create landmark right in front of robot (in the robot frame)
    Landmark lm_r(poses->back().range *0.5, 0);
    // now transfer this landmark over to the world frame
    Eigen::Vector3d lm_r_homogeneous(lm_r.vec()[0], lm_r.vec()[1], 1);
    Eigen::Vector3d lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    Landmark lm_w(lm_w_homogeneous[0], lm_w_homogeneous[1]);
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 1);
    REQUIRE( obs.at(0).observation.theta == Approx(0.) );
    REQUIRE( obs.at(0).observation.range == Approx(poses->back().range * 0.5));

    // create landmark (in the robot frame)
    double lm_angle = (poses->back().field_of_view / 2.);
    double dist = poses->back().range;
    lm_r = Landmark(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame, using the util homogenize/dehomogenize functions
    lm_r_homogeneous = chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    lm_w = Landmark(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 1);
    REQUIRE( obs.at(0).observation.theta == Approx(lm_angle) );
    REQUIRE( obs.at(0).observation.range == Approx(dist));


    // create landmark (in the robot frame)
    lm_angle = (poses->back().field_of_view / 2.) * 1.01; // just nudge it outside of the field of view
    dist = poses->back().range;
    lm_r = Landmark(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame, using the util homogenize/dehomogenize functions
    lm_r_homogeneous = chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    lm_w = Landmark(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 0 );

    // create landmark (in the robot frame)
    lm_angle = -(poses->back().field_of_view / 2.); // other side of fov
    dist = poses->back().range / 2;
    lm_r = Landmark(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame, using the util homogenize/dehomogenize functions
    lm_r_homogeneous = chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    lm_w = Landmark(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 1);
    REQUIRE( obs.at(0).observation.theta == Approx(lm_angle) );
    REQUIRE( obs.at(0).observation.range == Approx(dist));

    // create landmark (in the robot frame)
    lm_angle = -(poses->back().field_of_view / 2.) * 0.7; // other side of fov
    dist = poses->back().range * 1.02;
    lm_r = Landmark(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame, using the util homogenize/dehomogenize functions
    lm_r_homogeneous = chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    lm_w = Landmark(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 0 );

    // create landmark (in the robot frame)
    lm_angle = -(poses->back().field_of_view / 2.) * 1.03; // other side of fov
    dist = poses->back().range;
    lm_r = Landmark(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame, using the util homogenize/dehomogenize functions
    lm_r_homogeneous = chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    lm_w = Landmark(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 0 );
  }

  SECTION("robot not at origin (in negative coordinates) - transfer landmark to robot") {
    poses->push_back(RobotPose(M_PI/4, 3., -3.)); // create robot at (3,-3) with 45deg. orientation

    // create landmark right in front of robot (in the robot frame)
    double lm_angle = (poses->back().field_of_view / 2.);
    double dist = poses->back().range;
    Landmark lm_r(dist* std::cos(lm_angle), dist * std::sin(lm_angle));
    // now transfer this landmark over to the world frame
    Eigen::Vector3d lm_r_homogeneous =  chameleon::util::HomogenizeLandmark<>(lm_r.vec());
    Eigen::Vector3d lm_w_homogeneous = poses->back().pose.matrix() * lm_r_homogeneous;
    Landmark lm_w(chameleon::util::DeHomogenizeLandmark<>(lm_w_homogeneous));
    map->clear();
    map->push_back(lm_w);
    RangeFinderObservationVector obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).time == 1);
    REQUIRE( obs.at(0).observation.theta == Approx(lm_angle) );
    REQUIRE( obs.at(0).observation.range == Approx(dist));
  }



}
