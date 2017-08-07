// Copyright 2017 Toyota Research Institute.  All rights reserved.
// This class implements a "persistence filter" as decscribed in
// "Towards Lifeling Feature-Based Mapping in Semi-Static Environments"
// Rosen et al, ICRA 2016
//
// This persistence filter is associated to a single feature, as such it outputs
// the posterior:
// p(X = 1 | Y_1:N)
#pragma once
#include <cstddef>
namespace chameleon
{

class PersistenceFilter {
public:
  PersistenceFilter();

  ///
  /// \brief Posterior
  /// \return the posterior P(X=1 | Y_1:N)
  ///
  double Posterior();

  ///
  /// \brief Posterior
  /// \param t > the time for which we wish to query the persistence belief
  /// \return the posterior P(X=1 | Y_1:N) at time t
  ///
  double Posterior(double t);

  ///
  /// \brief AddObservation
  /// \param y_i > boolean value indicating if the feature was detected or not
  ///
  void AddObservation(bool y_i, double P_M, double P_F);

private:

  ///
  /// \brief Likelihood
  /// computes the measurement likelihood P(Y_1:N | t_N)
  /// \param t > timestep for which we wish to know the likelihood
  /// \return measurement likelihood
  ///
  double Likelihood(double t);

  ///
  /// \brief Evidence
  /// computes the evidence, P(Y_1:N)
  /// \return P(Y_1:N)
  ///
  double Evidence();

  std::vector<bool> obseravations_;  // sequence of reported observations of this feature
};

}  // namespace chameleon
