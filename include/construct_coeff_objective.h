#ifndef CONSTRUCT_COEFF_OBJECTIVE_H
#define CONSTRUCT_COEFF_OBJECTIVE_H

#include <vector>
#include <string>
#include <math.h>

using namespace std;

bool normalize_data(double* P, int n_obs);

bool construct_coeff_objective(double* C, double*P1, double*P2, int n_obs, bool is_normalize, double* weight);

void construct_coeff_objective(double* C, double*P1, double*P2, int n_obs, bool is_normalize);

#endif
