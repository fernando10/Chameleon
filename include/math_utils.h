// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace elninho
{

///
/// \brief AngleWraparound
/// \param angle : input angle in radians
/// \return angle in the [-PI PI) range
///
inline double AngleWraparound(const double angle) {
  double ret_angle = angle;
  while (ret_angle < M_PI) {
    ret_angle += 2 * M_PI;
  }
  while (ret_angle >= M_PI) {
    ret_angle -= 2 * M_PI;
  }
  return ret_angle;
}

///
/// \brief Deg2Rad
/// \param deg : angle in degrees
/// \return angle in radians
///
inline double Deg2Rad(const double deg) {
  return AngleWraparound(((deg / 180.0) * M_PI));
}

///
/// \brief Rad2Deg
/// \param rad : angle in radians
/// \return angle in degrees
///
inline double Rad2Deg(const double rad) {
  return (AngleWraparound(rad) * 180.0) / M_PI;
}

inline void SampleGaussian(const double mean, const double covariance, const double dim) {

}

}  // namespace elninho
