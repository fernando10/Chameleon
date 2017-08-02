// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <memory>
#include <Eigen/Core>
#include "fmt/string.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

namespace chameleon
{
namespace util
{

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args )
{
    return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

template<typename T = double>
inline Eigen::Matrix<T, 3, 1> HomogenizeLandmark(Eigen::Matrix<T, 2, 1> lm) {
  return Eigen::Matrix<T, 3, 1>(lm.x(), lm.y(), (T)1.);
}

template<typename T = double>
inline Eigen::Matrix<T, 2, 1> DeHomogenizeLandmark(Eigen::Matrix<T, 3, 1> lm_h) {
  if (lm_h[2] > (T)1e-6) {
    return Eigen::Matrix<T, 2, 1>(lm_h[0] / lm_h[2], lm_h[1] / lm_h[2]);
  }
  else {
    return Eigen::Matrix<T, 2, 1>::Zero();
  }
}

} // namespace util
} // namespace chameleon
