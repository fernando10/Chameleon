// Copyright 2017 Toyota Research Institute.  All rights reserved.
//

#include "chameleon/persistence_filter.h"
#include "glog/logging.h"

namespace chameleon
{

PersistenceFilter::PersistenceFilter(){
  Reset();
}

void PersistenceFilter::Reset() {
  VLOG(1) << "Resetting filter";
  t = 0;
  N = 0;
  meas_likelihood = 1;
  partial_evidence = 0;
  evidence = 1;
}

void PersistenceFilter::AddObservation(bool y_i, double P_M, double P_F) {
  // Compute partial evidence L(Y_{1:N+1})

  // Compute the likelihood p(Y_{1:N+1} | t_{N+1})

  // Compute the evidence p(Y_{1:N+1}

  // Update N
}

void PersistenceFilter::Predict() {
  // compute the posterior persistence probability
  // p(X_t = 1 | Y_{1:N}), t in [t_N, infty)

}


}
