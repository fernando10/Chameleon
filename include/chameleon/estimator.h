// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include "ceres/ceres.h"
#include "ceres/problem.h"
#include <Eigen/Core>
#include "chameleon/types.h"
#include "chameleon/util.h"

namespace chameleon
{
namespace ceres
{

class Estimator {
public:
  Estimator();
private:
  std::unique_ptr<ceres::Problem> ceres_problem_;
  ::ceres::Problem::Options ceres_options_;
};

}  // namespace ceres
}  // namespace chameleon
