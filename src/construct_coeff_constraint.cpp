#include "construct_coeff_constraint.h"

void construct_coeff_constraint(std::vector<Eigen::Matrix<double, 12, 12>>& A, std::vector<double>& b)
{
    A.clear();
    b.clear();
    
    int e11_idx = 0;
    int e21_idx = 1;
    int e31_idx = 2;

    int e12_idx = 3;
    int e22_idx = 4;
    int e32_idx = 5;

    int e13_idx = 6;
    int e23_idx = 7;
    int e33_idx = 8;

    int t1_idx = 9;
    int t2_idx = 10;
    int t3_idx = 11;

    // t^T * E = 0; E*E^T = [t]x * [t]x^T
    // #1
    Eigen::Matrix<double, 12, 12> t;
    t.setZero();
    t(e11_idx, e11_idx) = 1;
    t(e12_idx, e12_idx) = 1;
    t(e13_idx, e13_idx) = 1;
    t(t3_idx, t3_idx) = -1;
    t(t2_idx, t2_idx) = -1;
    A.push_back(t);
    b.push_back(0.0);

    // #2
    t.setZero();
    t(e21_idx, e21_idx) = 1;
    t(e22_idx, e22_idx) = 1;
    t(e23_idx, e23_idx) = 1;
    t(t3_idx, t3_idx) = -1;
    t(t1_idx, t1_idx) = -1;
    A.push_back(t);
    b.push_back(0.0);

    // #3
    t.setZero();
    t(e31_idx, e31_idx) = 1;
    t(e32_idx, e32_idx) = 1;
    t(e33_idx, e33_idx) = 1;
    t(t2_idx, t2_idx) = -1;
    t(t1_idx, t1_idx) = -1;
    A.push_back(t);
    b.push_back(0.0);

    // #4
    t.setZero();
    t(e11_idx, e21_idx) = 1;
    t(e12_idx, e22_idx) = 1;
    t(e13_idx, e23_idx) = 1;
    t(t2_idx, t1_idx) = 1;
    t = (t+t.transpose()).eval();
    A.push_back(t);
    b.push_back(0.0);

    // #5
    t.setZero();
    t(e11_idx, e31_idx) = 1;
    t(e12_idx, e32_idx) = 1;
    t(e13_idx, e33_idx) = 1;
    t(t3_idx, t1_idx) = 1;
    t = (t+t.transpose()).eval();
    A.push_back(t);
    b.push_back(0.0);

    // #6
    t.setZero();
    t(e21_idx, e31_idx) = 1;
    t(e22_idx, e32_idx) = 1;
    t(e23_idx, e33_idx) = 1;
    t(t3_idx, t2_idx) = 1;
    t = (t+t.transpose()).eval();
    A.push_back(t);
    b.push_back(0.0);

    // translation vector
    // #7
    t.setZero();
    t(t1_idx, t1_idx) = 1;
    t(t2_idx, t2_idx) = 1;
    t(t3_idx, t3_idx) = 1;
    A.push_back(t);
    b.push_back(1.0);

    return;
}