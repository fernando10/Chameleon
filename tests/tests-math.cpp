// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "math_utils.h"

using namespace elninho;

TEST_CASE("AngleWraparound", "[math]") {
  REQUIRE( AngleWraparound(5 * M_PI) == -M_PI );
  REQUIRE( AngleWraparound(0) == 0 );
  REQUIRE( AngleWraparound(M_PI) == -M_PI );
  REQUIRE( AngleWraparound(-M_PI) == -M_PI );
  REQUIRE( AngleWraparound(-M_PI * 20) == 0 );
}

TEST_CASE("Deg2Rad", "[math]") {
  REQUIRE ( Deg2Rad(0) == 0);
  REQUIRE ( Deg2Rad(-45) == Approx(-M_PI/4));
  REQUIRE ( Deg2Rad(180) == Approx(M_PI));
  REQUIRE ( Deg2Rad(-180) == Approx(-M_PI));
  REQUIRE ( Deg2Rad(360) == Approx(2*M_PI));
}
