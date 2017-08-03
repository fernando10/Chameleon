// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once
#include "ceres/ceres.h"
#include <Eigen/Core>
#include "chameleon/types.h"
#include <memory>

namespace chameleon
{
namespace ceres
{
struct OdometryCostFunction {
  OdometryCostFunction(const OdometryMeasurement& measurement, const OdometryCovariance &cov):
    measurement(measurement), inv_sqrt_cov(cov){
  }

  template<typename T>
  bool operator()(const T* const T1, const T* const T2, T* residual) const {
    // build odometry residual
    const Eigen::Map<const Sophus::SE2Group<T>>T_WS_prev(T1);
    const Eigen::Map<const Sophus::SE2Group<T>>T_WS_current(T2);

    Eigen::Map<Eigen::Matrix<T, OdometryMeasurement::kMeasurementDim, 1>>error(residual);

    T theta1 = AngleWraparound<T>(::ceres::atan2(T_WS_current.translation().y() - T_WS_prev.translation().y(),
                              T_WS_current.translation().x() - T_WS_prev.translation().x()) - T_WS_prev.so2().log());
    T translation = (T_WS_current.translation() - T_WS_prev.translation()).norm();
    T theta2 = AngleWraparound<T>(T_WS_current.so2().log() - T_WS_prev.so2().log() - theta1);

    // hardcoded for now
    // TODO: use kConsts for the component indexing
    error[0] = theta1 - (T)measurement.theta_1;
    error[1] = translation - (T)measurement.translation;
    error[2] = theta2 - (T)measurement.theta_2;

    error = inv_sqrt_cov.template cast<T>() * error;
    return true;
  }

  const OdometryMeasurement measurement;
  const OdometryCovariance inv_sqrt_cov;
};

struct RangeFinderObservationCostFunction {
  RangeFinderObservationCostFunction(const RangeFinderObservation& measurement,
                                     const RangeFinderCovariance cov): obs(measurement),
    inv_sqrt_cov(cov) {
  }

  template <typename T>
  bool operator()(const T* const T_WS_raw, const T* const LM_W_raw, T* residual) const {
    const Eigen::Map<const Sophus::SE2Group<T>>t_ws(T_WS_raw);
    const Eigen::Map<const Eigen::Matrix<T, Landmark::kLandmarkDim, 1>> lm_w(LM_W_raw);

    Eigen::Map<Eigen::Matrix<T, RangeFinderReading::kMeasurementDim, 1>> error(residual);

    // transfer landmark to robot frame
    Eigen::Matrix<T,  Landmark::kLandmarkDim, 1> lm_r =
        util::DeHomogenizeLandmark<T>(t_ws.matrix().inverse() * util::HomogenizeLandmark<T>(lm_w));

    // get the range and bearing information
    T range_pred = lm_r.norm();
    T bearing_pred = ::ceres::atan2(lm_r.y(), lm_r.x());
    error[RangeFinderReading::kIndexRange] = range_pred - (T)obs.observation.range;
    error[RangeFinderReading::kIndexBearing] = bearing_pred - (T)obs.observation.theta;

    error = inv_sqrt_cov.template cast<T>() * error;
    return true;
  }

  const RangeFinderObservation obs;
  const RangeFinderCovariance inv_sqrt_cov;
};
}  // namespace ceres
}  // namespace chameleon
