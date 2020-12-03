#ifndef NPT_POSE_H
#define NPT_POSE_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include "sdpa_call.h"
#include "construct_coeff_objective.h"

using namespace std;
using namespace Eigen;

int find_largest_ele_in_complex_vector(Eigen::VectorXcd v);

void npt_pose(double* P1, double* P2, double* C, int n_obs, 
    Eigen::Matrix<double, 12, 12>& X_sol, Eigen::Matrix3d& E, bool is_normalize=true);

#endif
