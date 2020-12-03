#ifndef CREATE_2D2D_EXPERIMENT_H
#define CREATE_2D2D_EXPERIMENT_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void generateRandomR(Eigen::Matrix3d& R);

void generateBoundedR(Eigen::Matrix3d& R, double bound);

void addNoise(Eigen::Vector3d& v_noisy, Eigen::Vector3d& v_clean, double focal_length, double pixel_noise);

void generateRandomPoint(Eigen::Vector3d& point, double minDepth, double maxDepth);

void create2D2DExperiment(int pt_number, int cam_number, double noise, double outlier_fraction, 
        std::vector<Eigen::Matrix<double, 6, 1>>& v1, std::vector<Eigen::Matrix<double, 6, 1>>& v2, 
        Eigen::Matrix3d& R, Eigen::Vector3d& t, double focal_length = 800.0);

void create2D2DExperiment(int pt_number, double noise, double outlier_fraction, 
        std::vector<Eigen::Matrix<double, 3, 1>>& v1, std::vector<Eigen::Matrix<double, 3, 1>>& v2, 
        Eigen::Matrix3d& R, Eigen::Vector3d& t, double focal_length = 800.0);

#endif
