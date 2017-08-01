// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "chameleon/estimator.h"

namespace chameleon
{
namespace ceres
{

Estimator::Estimator() {
  ceres_problem_ = util::make_unique<ceres::Problem>();
}

}  // namespace ceres
}  // namespace chameleon
