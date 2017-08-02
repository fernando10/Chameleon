// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <sophus/se2.hpp>

namespace Sophus
{
namespace chameleon
{

struct AutoDiffLocalParamSE2 {
  template <typename T>
  bool operator()(const T* x_raw, const T* delta_raw, T* x_plus_delta_raw) const {
    const Eigen::Map<const Sophus::SE2Group<T>> x(x_raw);
    const Eigen::Map<const Eigen::Matrix<T, Sophus::SE2Group<T>::DoF, 1>>delta(delta_raw);
    Eigen::Map<Sophus::SE2Group<T>> x_plus_delta(x_plus_delta_raw);
    x_plus_delta = x * Sophus::SE2Group<T>::exp(delta);
    return true;
  }
};

}
}
