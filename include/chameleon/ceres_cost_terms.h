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

// this cost function takes in as a measurement an 'odometry' reading in the form multiple forward velocity + omega
// usually real data will take this form
struct OdometryReadingCostFunction {
  OdometryReadingCostFunction(const OdometryObservationVector measurements, const OdometryCovariance& cov,
                              double dt):
    measurements(measurements), inv_sqrt_cov(cov), dt(dt) {}

  template<typename T>
  bool operator()(const T* const T1, const T* const T2, T* residual) const {
    const Eigen::Map<const Sophus::SE2Group<T>>T_WS_prev(T1);
    const Eigen::Map<const Sophus::SE2Group<T>>T_WS_current(T2);

    // error is predicted delta pose - estimated delta pose
    Eigen::Map<Eigen::Matrix<T, Sophus::SE2Group<T>::DoF, 1>>error(residual);


    Sophus::SE2Group<T> pose = T_WS_prev;
    for (size_t i = 0; i < measurements.size(); ++i) {
      // do simple euler integration
      T theta = T(dt) * (T)measurements.at(i).observation.omega;  // rad
      T dx = T(dt) * (T)measurements.at(i).observation.velocity * ::ceres::cos(pose.so2().log());  // m
      T dy = T(dt) * (T)measurements.at(i).observation.velocity * ::ceres::sin(pose.so2().log());  // m

      pose.so2() = Sophus::SO2Group<T>(AngleWraparound<T>(pose.so2().log() + theta));
      pose.translation().x() = pose.translation().x() + dx;
      pose.translation().y() = pose.translation().y() + dy;
    }

    // now compute the residual
    Eigen::Matrix<T, 3, 1> res = (pose.inverse() * T_WS_current).log();
    error[0] = res[0]; // x
    error[1] = res[1]; // y
    error[2] = res[2]; // theta

    error = inv_sqrt_cov.template cast<T>() * error;
    return true;
  }

  const OdometryObservationVector measurements;
  const OdometryCovariance inv_sqrt_cov;
  const double dt;
};


// this cost function takes in as a measurement an 'odometry' reading in the form a rotation + translation + rotation
// usually simulated data will take this form
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
    T bearing_pred = AngleWraparound<T>(::ceres::atan2(lm_r.y(), lm_r.x()));
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
