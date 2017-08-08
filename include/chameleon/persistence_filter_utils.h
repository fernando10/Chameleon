// This file is from the repository https://github.com/david-m-rosen/persistence_filter
//==== Copyright and License ====
// The C++ and Python implementations of the Persistence Filter contained herein are copyright (C) 2016 by David M. Rosen,
// and are distributed under the terms of the GNU General Public License (GPL) version 3 (or later).
#pragma once

namespace chameleon
{
/**This function implements the survival function for the general-purpose survival-time prior developed in the RSS workshop paper "Towards Lifelong Feature-Based Mapping in Semi-Static Environments".*/
double log_general_purpose_survival_function(double t, double lambda_l, double lambda_u);

/**Computes log(x + y) from log(x), log(y) in a numerically stable way.*/
double logsum(double logx, double logy);

/**Computes log(x - y) from log(x), log(y) in a numerically stable way.  Note that here we require x > y.*/
double logdiff(double logx, double logy);

} // namespace chameleon
