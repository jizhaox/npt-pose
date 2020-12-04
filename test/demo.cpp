#include <iostream>
#include <chrono>
#include "construct_coeff_constraint.h"
#include "create2D2DExperiment.h"
#include "essential_matrix.h"
#include "npt_pose.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    // parameters for synthetic scene
    int cam_number = 1;
    int pt_number = 100;
    double noise_level = 1.0;
    double outlier_fraction = 0.0;

    std::vector<Eigen::Matrix<double, 6, 1>> v1, v2; 
    Eigen::Matrix3d R_gt, E_gt;
    Eigen::Vector3d T_gt;
    double* P1 = new double[pt_number*3];
    double* P2 = new double[pt_number*3];

    std::vector<Eigen::Matrix<double, 12, 12>> A;
    std::vector<double> b;
    double* C = new double[81];
    Eigen::Matrix<double, 12, 12> X_sol;
    Eigen::Matrix3d E_est;

    int n_times = 100;
    for (int i = 0; i < n_times; i++)
    {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "i: " << i << std::endl;
        // prepare data
        create2D2DExperiment(pt_number, cam_number, noise_level, outlier_fraction, v1, v2, R_gt, T_gt);
        construct_essential_matrix(R_gt, T_gt, E_gt);
        for (int i = 0; i < pt_number; i ++)
        {
            P1[i*3] = v1[i](0);
            P1[i*3+1] = v1[i](1);
            P1[i*3+2] = v1[i](2);
            P2[i*3] = v2[i](0);
            P2[i*3+1] = v2[i](1);
            P2[i*3+2] = v2[i](2);
        }

        // do the job
        // timer begin
        std::chrono::time_point<std::chrono::system_clock> time_start = std::chrono::system_clock::now();
        npt_pose(P1, P2, C, pt_number, X_sol, E_est, true);
        //timer end
        std::chrono::time_point<std::chrono::system_clock> time_end = std::chrono::system_clock::now();
        std::chrono::duration<double> time_spend = time_end - time_start;
        std::cout << "Runtime: " << time_spend.count()*1000.0 << " ms" << std::endl;

        // show the results
        if (1)
        {
            std::cout << "ground truth" << std::endl;
//            std::cout << "R = " << std::endl << R_gt << std::endl;
//            std::cout << "T = " << std::endl << T_gt.transpose() << std::endl;
            std::cout << "E = " << std::endl << E_gt << std::endl;
            std::cout << "optimization" << std::endl;
            std::cout << "E = " << std::endl << E_est << endl;
        }
    }
    delete[] P1;
    delete[] P2;
    delete[] C;

    return 1;
}

