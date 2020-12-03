#ifndef ESSENTIAL_MATRIX_H
#define ESSENTIAL_MATRIX_H

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void construct_essential_matrix(Eigen::Matrix3d& R, Eigen::Vector3d& T, Eigen::Matrix3d& E);

#endif
