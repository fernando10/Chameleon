// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "types.h"
#include "util.h"

namespace elninho
{

enum class WorldTypes {
  RectangularField,
  StraightLine,
  TwoParallelLines,
  RandomTrees
};

class WorldGenerator {
 public:
  ///
  /// \brief WorldGenerator
  /// Default Constructor
  WorldGenerator(){}

  GenerateWorld(WorldTypes type);

private:

  ///
  /// \brief GenerateRectangularField
  /// Generates a rectangle of landmarks centered at the origin
  void GenerateRectangularField();

  const double kLandmarkDensity = 0.5;  // landmarks / meter
  const double kFieldWidth = 10.; // [m]
  const double kFieldLength = 5.; // [m]


};

} // namespace elninho
