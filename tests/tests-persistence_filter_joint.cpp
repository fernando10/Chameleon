// Copyright 2017 Toyota Research Institute.  All rights reserved.
//
#include "catch.hpp"

#include "chameleon/persistence_filter.h"
#include "chameleon/persistence_filter_utils.h"

#include <functional>
#include <iostream>
#include <gsl/gsl_sf_exp.h>

using namespace chameleon;

TEST_CASE("Joint Persistence Filter") {

  double lambda_u = 1;
  double lambda_l = .01;

  double P_M = .2;
  double P_F = .01;

  std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);

  auto S_T = [=](double t) { return gsl_sf_exp(logS_T(t)); };

  PersistenceFilter filter(logS_T);

  // 2 landmarks l1 and l2
  // weights w_11, w_12, w_21, w_22
  // 2 sets of observations: {j_11 = 0, j_12 = 1} at time t_1, then {j_21 = 1, j_22 = 1} at time t_2

  double t_1 = 1.0;
  double t_2 = 2.0;

  // INCORPORATE FIRST OBSERVATION {j_11 = 0, j_12 = 1} at t_1 = 1.0

  //filter.update(false, t_1, P_M, P_F);  // Update the filter
  //double filter_posterior1 = filter.predict(t_1);  // Compute posterior prediction

  //The likelihood p(j_11 = 0, j_12 = 1 | T1 >= t_1, T2 >= t_2) =  P_M * (1 - P_M)
  double pY1_t1 = P_M * (1 - P_M);

  //The evidence probability p(j_11 = 0, j_12 = 1) =
  //p(j_11 = 0, j_12 = 1 | T1 >= t1, T2 >= t1) * P(T1 >= t1, T2 >= t1) +
  //p(j_11 = 0, j_12 = 1 | T1 >= t1, T2 < t1) * P(T1 >= t1, T2 < t1) +
  //p(j_11 = 0, j_12 = 1 | T1 < t1, T2 >= t1) * P(T1 < t1, T2 >= t1) +
  //p(j_11 = 0, j_12 = 1 | T1 < t1, T2 < t1) * P(T1 < t1, T2 < t1)
  //The evidence probability p(y_1 = 0) = p(y_1 = 0 | T >= t_1) * p(T >= t_1) + p(y_1 = 0 | T < t_1) * p(T < t_1)
  //double pY1 = P_M * S_T(t_1) + (1 - P_F) * (1 - S_T(t_1));
  double pY1 = P_M * (1 - P_M) * S_T(t_1) * S_T(t_1) +
               P_M * P_F * S_T(t_1) * (1 - S_T(t_1)) +
               (1 - P_F) * (1 - P_M) * (1 - S_T(t_1)) * S_T(t_1) +
               (1 - P_F) * P_F * (1 - S_T(t_1)) * (1 - S_T(t_1));

  //Compute the posterior p(Theta_{t_1} = 1 | j_11 = 0, j_12 = 1) = p(j_11 = 0, j_12 = 1 | T1 >= t_1, T2 >= t_1) / p(j_11 = 0, j_12 = 1) * p(T1 >= t_1, T2 >= t_2)
  double posterior1 = (pY1_t1 / pY1) * (S_T(t_1) * S_T(t_1));

  REQUIRE(pY1_t1 == Approx(filter.likelihood()));
  REQUIRE(pY1 == Approx(filter.evidence()));
  REQUIRE(posterior1 == Approx(filter_posterior1));

  // INCORPORATE SECOND OBSERVATION y_2 = 1 at t_2 = 2.0

  filter.update(true, t_2, P_M, P_F);  // Update the filter
  double filter_posterior2 = filter.predict(t_2);  // Compute posterior prediction

  //The likelihood p(y_1 = 0, y_2 = 1 | T >= t_2)
  double pY2_t2 = P_M * (1-P_M);

  // The evidence probability p(y_1 = 0, y_2 = 1) =
  // p(y_1 = 0, y_2 = 1 | T > t_2) * p(T > t_2) +
  // p(y_1 = 0, y_2 = 1 | t_1 <= T < t_2) * p(t_1 <= T < t_2) +
  // p(y_1 = 0, y_2 = 1 | T < t_1) * p(t < t_1)

  double pY2 = P_M * (1 - P_M) * S_T(t_2)
    + P_M * P_F * (S_T(t_1) - S_T(t_2))
    + (1 - P_F) * P_F * (1 - S_T(t_1));

  // Compute the posterior p(X_{t_2} = 2 | y_1 = 0, y_2 = 1) = p(y_1 = 0, y_2 = 1 | T >= t_2) / p(y_1 = 0, y_2 = 1) * p(T >= t_2)

  double posterior2 = (pY2_t2 / pY2) * S_T(t_2);

  REQUIRE(pY2_t2 == Approx(filter.likelihood()));
  REQUIRE(pY2 == Approx(filter.evidence()));
  REQUIRE(posterior2 == Approx(filter_posterior2));

  // INCORPORATE THIRD OBSERVATION y_3 = 0 at t_3 = 3.0

  filter.update(false, t_3, P_M, P_F);  // Update the filter
  double filter_posterior3 = filter.predict(t_3);  // Compute posterior prediction


  //The likelihood p(y_1 = 0, y_2 = 1 | T >= t_2)
  double pY3_t3 = P_M * (1-P_M) * P_M;

  // The evidence probability p(y_1 = 0, y_2 = 1, y_3 = 0) =
  // p(y_1 = 0, y_2 = 1, y_3 = 0 | T > t_3) * p(T > t_3) +
  // p(y_1 = 0, y_2 = 1, y_3 = 0 | t_2 <= T < t_3) * p(t_2 <= T < t_3) +
  // p(y_1 = 0, y_2 = 1, y_3 = 0 | t_1 <= T < t_2) * p(t_1 <= T < t_2) +
  // p(y_1 = 0, y_2 = 1, y_3 = 0 | T < t_1) * p(t < t_1)

  double pY3 = P_M * (1 - P_M) * P_M * S_T(t_3)
    + P_M * (1 - P_M) * (1 - P_F) * (S_T(t_2) - S_T(t_3))
    + P_M * P_F * (1 - P_F) * (S_T(t_1) - S_T(t_2))
    + (1 - P_F) * P_F * (1 - P_F) * (1 - S_T(t_1));

    // Compute the posterior p(X_{t_2} = 2 | y_1 = 0, y_2 = 1) = p(y_1 = 0, y_2 = 1 | T >= t_2) / p(y_1 = 0, y_2 = 1) * p(T >= t_2)

  double posterior3 = (pY3_t3 / pY3) * S_T(t_3);

  REQUIRE(pY3_t3 == Approx(filter.likelihood()));
  REQUIRE(pY3 == Approx(filter.evidence()));
  REQUIRE(posterior3 == Approx(filter_posterior3));
}
