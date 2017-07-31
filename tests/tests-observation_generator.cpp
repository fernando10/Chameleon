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
    REQUIRE( obs.at(0).timestamp == 0.);
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
    poses->push_back(RobotPose(M_PI/4, 3., 4.)); // create robot at (3,4) with 45deg. orientation
    map->push_back(Landmark(2, 2));
    RangeFinderObservationVector obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 0 );

    map->push_back(Landmark(4, 5));
    obs = observation_generator.GenerateObservations(1);
    REQUIRE( obs.size() == 1 );
    REQUIRE( obs.at(0).timestamp == 1);
  }



}
