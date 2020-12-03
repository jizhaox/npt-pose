#include "npt_pose.h"

int find_largest_ele_in_complex_vector(Eigen::VectorXcd v)
{
    int n = v.rows() * v.cols();
    double re = v[0].real();
    double im = v[0].imag();
    double value_max = re*re + im*im;
    int idx = 0;
    for (int i = 1; i < n; i++)
    {
        re = v[i].real();
        im = v[i].imag();
        double tmp = re*re + im*im;
        if (tmp > value_max)
        {
            value_max = tmp;
            idx = i;
        }
    }
    return idx;
}

void npt_pose(double* P1, double* P2, double* C, int n_obs, 
    Eigen::Matrix<double, 12, 12>& X_sol, Eigen::Matrix3d& E, bool is_normalize)
{
    // construct coefficient matrix of objective
    construct_coeff_objective(C, P1, P2, n_obs, is_normalize);
    
    // SDP optimization
    SDPA npt_problem;
    npt_problem.setParameterType(SDPA::PARAMETER_DEFAULT);

    // number of constraints
    int mDIM   = 7;
    int nBlock = 1;
    npt_problem.inputConstraintNumber(mDIM);
    npt_problem.inputBlockNumber(nBlock);
    npt_problem.inputBlockSize(1, 12);
    npt_problem.inputBlockType(1, SDPA::SDP);

    npt_problem.initializeUpperTriangleSpace();

    for (int i = 1; i <= 6; i++)
    {
        npt_problem.inputCVec(i, 0);
    }
    npt_problem.inputCVec(7,-1);
    
    // Input F0
    for (int i = 0; i < 9; i++)
    {
        for (int j = i; j < 9; j++)
        {
            npt_problem.inputElement(0, 1, i+1, j+1, -C[i*9+j]); 
        }
    }

    // Input F_1 -- F_7
    int e11 = 1;
    int e21 = 2;
    int e31 = 3;
    int e12 = 4;
    int e22 = 5;
    int e32 = 6;
    int e13 = 7;
    int e23 = 8;
    int e33 = 9;
    int t1 = 10;
    int t2 = 11;
    int t3 = 12;

    npt_problem.inputElement(1, 1, e11, e11, -1);
    npt_problem.inputElement(1, 1, e12, e12, -1);
    npt_problem.inputElement(1, 1, e13, e13, -1);
    npt_problem.inputElement(1, 1, t3, t3, 1);
    npt_problem.inputElement(1, 1, t2, t2, 1);

    npt_problem.inputElement(2, 1, e21, e21, -1);
    npt_problem.inputElement(2, 1, e22, e22, -1);
    npt_problem.inputElement(2, 1, e23, e23, -1);
    npt_problem.inputElement(2, 1, t1, t1, 1);
    npt_problem.inputElement(2, 1, t3, t3, 1);

    npt_problem.inputElement(3, 1, e31, e31, -1);
    npt_problem.inputElement(3, 1, e32, e32, -1);
    npt_problem.inputElement(3, 1, e33, e33, -1);
    npt_problem.inputElement(3, 1, t1, t1, 1);
    npt_problem.inputElement(3, 1, t2, t2, 1);

    npt_problem.inputElement(4, 1, e11, e21, -1);
    npt_problem.inputElement(4, 1, e12, e22, -1);
    npt_problem.inputElement(4, 1, e13, e23, -1);
    npt_problem.inputElement(4, 1, t1, t2, -1);

    npt_problem.inputElement(5, 1, e11, e31, -1);
    npt_problem.inputElement(5, 1, e12, e32, -1);
    npt_problem.inputElement(5, 1, e13, e33, -1);
    npt_problem.inputElement(5, 1, t1, t3, -1);

    npt_problem.inputElement(6, 1, e21, e31, -1);
    npt_problem.inputElement(6, 1, e22, e32, -1);
    npt_problem.inputElement(6, 1, e23, e33, -1);
    npt_problem.inputElement(6, 1, t2, t3, -1);

    npt_problem.inputElement(7, 1, t1, t1, -1);
    npt_problem.inputElement(7, 1, t2, t2, -1);
    npt_problem.inputElement(7, 1, t3, t3, -1);

    npt_problem.initializeUpperTriangle();

    // perform optimization
    npt_problem.initializeSolve();
    npt_problem.solve();

    // extract solution
    double* X = npt_problem.getResultYMat(1);
    Eigen::Matrix<double, 9, 9> X_mat;
    int dim = npt_problem.getBlockSize(1);
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            int idx = i*dim+j;
            X_mat(i, j) = X[idx];
        }
    }
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            int idx = i*dim+j;
            X_sol(i, j) = X[idx];
        }
    }

    // extract essential matrix
    EigenSolver<Eigen::Matrix<double,9,9>> es(X_mat);
    Eigen::VectorXcd ev = es.eigenvalues();
    int idx_largest = find_largest_ele_in_complex_vector(es.eigenvalues());
    Eigen::VectorXcd v = es.eigenvectors().col(idx_largest);
    Eigen::MatrixXd v_real = v.real();

    v_real.normalize();
    v_real = v_real * sqrt(2.0);
    for (int j = 0; j < 3; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            E(i, j) = v_real(i*3 + j);
        }
    }

    return;
}
