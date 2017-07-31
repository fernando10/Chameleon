// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "chameleon/math_utils.h"

using namespace chameleon;

TEST_CASE("AngleWraparound", "[math]") {
  REQUIRE( AngleWraparound(5 * M_PI) == -M_PI );
  REQUIRE( AngleWraparound(0) == 0 );
  REQUIRE( AngleWraparound(M_PI) == -M_PI );
  REQUIRE( AngleWraparound(-M_PI) == -M_PI );
  REQUIRE( AngleWraparound(-M_PI * 20) == 0 );
}
