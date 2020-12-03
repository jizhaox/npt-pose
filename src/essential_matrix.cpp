#include "essential_matrix.h"

void construct_essential_matrix(Eigen::Matrix3d& R, Eigen::Vector3d& T, Eigen::Matrix3d& E)
{
    T.normalize();
    Eigen::Matrix3d T_skew;
    T_skew << 0, -T(2), T(1),
              T(2), 0, -T(0),
              -T(1), T(0), 0;
    E = T_skew * R;
    return;
}