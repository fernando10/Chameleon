// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#pragma once

#include <random>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace chameleon
{

///
/// \brief AngleWraparound
/// \param angle : input angle in radians
/// \return angle in the [-PI PI) range
template<typename T>
inline T AngleWraparound(const T angle) {
  T ret_angle = angle;
  while (ret_angle <T( M_PI)) {
    ret_angle += T(2 * M_PI);
  }
  while (ret_angle >= T(M_PI)) {
    ret_angle -= T(2 * M_PI);
  }
  return ret_angle;
}

///
/// \brief Deg2Rad
/// \param deg : angle in degrees
/// \return angle in radians
///
inline double Deg2Rad(const double deg) {
  return AngleWraparound<double>(((deg / 180.0) * M_PI));
}

///
/// \brief Rad2Deg
/// \param rad : angle in radians
/// \return angle in degrees
///
inline double Rad2Deg(const double rad) {
  return (AngleWraparound<double>(rad) * 180.0) / M_PI;
}

///
/// \brief Square
/// \param x variable to be squared
/// \return
///
inline double Square(const double x) {
  return x * x;
}

template<typename Range1, typename Range2, typename OutputIterator>
void cartesian_product(Range1 const &r1, Range2 const &r2, OutputIterator out) {
    using std::begin; using std::end;

    for (auto i = begin(r1); i != end(r1); ++i) {
        for (auto j = begin(r2); j != end(r2); ++j) {
            *out++ = std::make_tuple(*i, *j);
        }
    }
}


struct Distribution {
  Distribution(Eigen::VectorXd mean, Eigen::MatrixXd cov) : mean(mean), cov(cov){}

  Eigen::VectorXd mean;
  Eigen::MatrixXd cov;
};

inline Eigen::Matrix2d Covariance2d(double sigma1, double sigma2, double corr) {
  return (Eigen::Matrix2d() << Square(sigma1),                 corr * sigma1 * sigma2,
                                                      corr * sigma1  * sigma2,    Square(sigma2)).finished();
}

///
/// \brief The MultivariateNormalVariable struct
/// Represents a multivariate normal and allows sampling the variable
/// see: https://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
///
struct MultivariateNormalVariable {
  MultivariateNormalVariable(const Eigen::MatrixXd& cov):
    MultivariateNormalVariable(cov, Eigen::VectorXd::Zero(cov.rows())) {}

  MultivariateNormalVariable(const Distribution& dist):
    MultivariateNormalVariable(dist.cov, dist.mean){}

  MultivariateNormalVariable(const Eigen::MatrixXd& cov, const Eigen::VectorXd& mean): mean(mean) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);
    A = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd operator()() const
  {
      //static std::mt19937 gen{ std::random_device{}() };
      static std::mt19937 gen{ 0 };  // make deterministic
      static std::normal_distribution<> dist;

      // transforms a zero-mean unit covariance normal to our mean and covariance
      return mean + A * Eigen::VectorXd(mean.size()).unaryExpr([&](double x) { return dist(gen); });
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd A;
};

}  // namespace chameleon
