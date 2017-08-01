// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <memory>
#include <Eigen/Core>
#include "glog/logging.h"
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

inline Eigen::Vector3d HomogenizeLandmark(Eigen::Vector2d lm) {
  return Eigen::Vector3d(lm.x(), lm.y(), 1.);
}

inline Eigen::Vector2d DeHomogenizeLandmark(Eigen::Vector3d lm_h) {
  if (lm_h[2] > 1e-4) {
    return Eigen::Vector2d(lm_h[0] / lm_h[2], lm_h[1] / lm_h[2]);
  }
  else {
    LOG(ERROR) << "Dehomogenizing point at infinity.";
    return Eigen::Vector2d::Zero();
  }
}

} // namespace util
} // namespace chameleon
