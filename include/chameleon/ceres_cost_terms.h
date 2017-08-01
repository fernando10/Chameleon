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
  bool operator()(const T* const T1, const T* const T2, T* residual) const {
    // build odometry residual
    Eigen::Map<Sophus::SE2Group<T>>T_WS_prev(T1);
    Eigen::Map<Sophus::SE2Group<T>>T_WS_current(T2);

    Eigen::Map<Eigen::Matrix<T, 3, 1>>error(residual);

    T theta1 = ::ceres::atan2(T_WS_current.translation().y() - T_WS_prev.translation().y(),
                             T_WS_current.translation().x() - T_WS_prev.translation().x()) - T_WS_prev.so2().log();
    T translation = (T_WS_current.translation() - T_WS_prev.translation()).norm();
    T theta2 = T_WS_current.so2().log() - T_WS_prev.so2().log() - theta1;

    error[0] = theta1 - measurement.theta_1.template cast<T>();
    error[1] = translation - measurement.translation.template cast<T>();
    error[2] = theta2 - measurement.theta_2.template cast<T>();

    error = error * covariance.template cast<T>();

    return true;
  }

  const OdometryMeasurement measurement;
  const OdometryCovariance covariance;
};
}  // namespace ceres
}  // namespace chameleon
