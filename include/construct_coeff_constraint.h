#ifndef CONSTRUCT_COEFF_CONSTRAINT_H
#define CONSTRUCT_COEFF_CONSTRAINT_H

#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void construct_coeff_constraint(std::vector<Eigen::Matrix<double, 12, 12>>& A, std::vector<double>& b);

#endif
