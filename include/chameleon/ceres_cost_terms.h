// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include "ceres/ceres.h"
#include <Eigen/Core>
#include "chameleon/types.h"

namespace chameleon
{
namespace ceres
{
struct OdometryCostFunction {
  OdometryCostFunction(const OdometryMeasurement& measurement, const OdometryCovariance &cov):
  measurement(measurement), covariance(cov){
  }

  template<typename T>
  bool operator()(const T* const pose_a, const T* const pose_b, T* residual) const {
    // build odometry residual

    return true;
  }

  const OdometryMeasurement measurement;
  const OdometryCovariance covariance;
};
}  // namespace ceres
}  // namespace chameleon
