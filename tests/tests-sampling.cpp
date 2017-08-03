// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"
#include "chameleon/math_utils.h"
using namespace chameleon;

TEST_CASE("Sampling a multivariage Gaussian many times should yeild the true mean and variance") {
  Eigen::Matrix<double, 1, 1> cov;
  cov(0,0) = 1;
  Eigen::Matrix<double, 1, 1> mean;
  mean(0, 0) = 0.;
  MultivariateNormalVariable random_var(cov, mean);  // zero mean, unit variance distribution
  static constexpr size_t kNumSamples = 1e3;
  Eigen::Matrix<double, kNumSamples, 1> values;

  for (size_t idx = 0; idx < kNumSamples; ++idx) {
    values[idx] = random_var()[0]; // sample the distribution
  }

  REQUIRE(values.mean() == Approx(10.));
}
