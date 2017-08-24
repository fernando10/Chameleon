// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include "chameleon/types.h"

namespace chameleon
{


class DataProviderBase {
public:
  DataProviderBase(){}

  ///
  /// \brief GetRobotData
  /// \param data data structure to be filled out by implementing classes
  /// \return true if data was filled out, false if no data was available
  ///
  virtual bool GetData(RobotData* const data) = 0;
};

}  // namespace chameleon
