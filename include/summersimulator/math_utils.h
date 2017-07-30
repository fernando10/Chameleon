// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <random>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace summer
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

///
/// \brief Square
/// \param x variable to be squared
/// \return
///
inline double Square(const double x) {
  return x * x;
}

///
/// \brief The MultivariateNormalVariable struct
/// Represents a multivariate normal and allows sampling the variable
/// see: https://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
///
struct MultivariateNormalVariable {
  MultivariateNormalVariable(const Eigen::MatrixXd& cov):
    MultivariateNormalVariable(cov, Eigen::VectorXd::Zero(cov.rows())) {}

  MultivariateNormalVariable(const Eigen::MatrixXd& cov, const Eigen::VectorXd& mean): mean(mean) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);
    A = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd operator()() const
  {
      static std::mt19937 gen{ std::random_device{}() };
      static std::normal_distribution<> dist;

      // transforms a zero-mean unit covariance normal to our mean and covariance
      return mean + A * Eigen::VectorXd(mean.size()).unaryExpr([&](double x) { return dist(gen); });
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd A;
};


}  // namespace summer
