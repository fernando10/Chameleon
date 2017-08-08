// This file is from the repository https://github.com/david-m-rosen/persistence_filter
//==== Copyright and License ====
// The C++ and Python implementations of the Persistence Filter contained herein are copyright (C) 2016 by David M. Rosen,
// and are distributed under the terms of the GNU General Public License (GPL) version 3 (or later).
//
// This class implements a "persistence filter" as decscribed in
// "Towards Lifeling Feature-Based Mapping in Semi-Static Environments"
// Rosen et al, ICRA 2016
//
// This persistence filter is associated to a single feature, as such it outputs
// the posterior:
// p(X = 1 | Y_{1:N})
#pragma once
#include <cstddef>
#include <gsl/gsl_sf_exp.h>
#include <functional>
#include <memory>
#include "glog/logging.h"
#include "fmt/format.h"
#include "fmt/ostream.h"

namespace chameleon
{

class PersistenceFilter
{
 protected:

  /** The absolute time (wall-time) at which this filter was initialized*/
  double init_time_;

  /** The time of the last observation*/
  double tN_;

  /** The natural logarithm of the likelihood probability p(Y_{1:N} | t_N)*/
  double logpY_tN_;

  /** The natural logarithm of the lower evidence sum L(Y_{1:N}). We initialize this running sum after the incorporation of the first observation.*/
  double* logLY_;

  /** The natural logarithm of the marginal (evidence) probability p(Y_{1:N})*/
  double logpY_;

  /** A function returning the natural logarithm of the survival function S_T() for the survival time prior p_T()*/
  std::function<double(double)> logS_;

  /** A function returning the natural logarithm of the time-shifted survival function S_T(t - initialization_time)*/
  std::function<double(double)> shifted_logS_;

  /** A helper function that computes the logarithm of the prior probability assigned to the range [t0, t1) by the shiftd survival time prior*/
  double shifted_logdF(double t1, double t0);


 public:
  /** One argument-constructor accepting a function that returns the logarithm of the survival function S_T() for the survival time prior p_T().*/
 PersistenceFilter(const std::function<double(double)>& log_survival_function, double initialization_time = 0.0) :
   init_time_(initialization_time),  tN_(initialization_time), logpY_tN_(0.0), logLY_(nullptr), logpY_(0.0), logS_(log_survival_function)

    {
      VLOG(2) << "Initializing persistence filter at start time: " << initialization_time;
      auto time_shifter = [](double time, double time_shift){ return time - time_shift; };
      shifted_logS_ = std::bind(logS_, std::bind(time_shifter, std::placeholders::_1, init_time_));
    }

  /** Updates the filter by incorporating a new detector output.  Here 'detector_output' is a boolean value output by the detector indicating whether the given feature was detected, 'observation_time' is the timestamp for the detection, and 'P_M' and 'P_F' give the detector's missed detection and false alarm probabilities for this observation, respectively.*/
  void update(bool detector_output, double observation_time, double P_M, double P_F);

  /** Compute the posterior feature persistence time p(X_t = 1 | Y_{1:N}) at time t >= tN (the time of the last observation).*/
  double predict(double prediction_time) const;

  /** Return the function computing the logarithm of the survival function.*/
  const std::function<double(double)>& logS() const
    {
      return logS_;
    }

  /** Return the function computing the logarithm of the shifted survival function*/
  const std::function<double(double)>& shifted_logS() const
    {
      return shifted_logS_;
    }

  /** Return the time of the last observation*/
  double last_observation_time() const
  {
    return tN_;
  }

  /** Return the absolute time (wall-time) at which this filter was initialized*/
  double initialization_time() const
  {
    return init_time_;
  }

  /** Return the likelihood probability p(Y_{1:N} | T >= t_N)*/
  double likelihood() const
  {
    return gsl_sf_exp(logpY_tN_);
  }

  /** Return the evidence probability p(Y_{1:N})*/
  double evidence() const
  {
    return gsl_sf_exp(logpY_);
  }

  /** Return the lower-sum probability L(Y_{1:N})*/
  double evidence_lower_sum() const
  {
    if(logLY_)
      return gsl_sf_exp(*logLY_);
    else
      return 0.0;
  }

  /** Nothing to do here*/
  ~PersistenceFilter() {}
};

typedef std::shared_ptr<PersistenceFilter> PersistenceFilterPtr;

}  // namespace chameleon
