#include "create2D2DExperiment.h"

// converted from some functions in openGV
// https://github.com/laurentkneip/opengv/tree/master/matlab/helpers

void generateRandomR(Eigen::Matrix3d& R)
{
    Eigen::Vector3d rpy;
    rpy(0) = M_PI * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    rpy(1) = M_PI * ( ((double) rand() / (RAND_MAX)) - 0.5);
    rpy(2) = M_PI * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);

    R =  AngleAxisd(rpy[2], Vector3d::UnitZ())
        *AngleAxisd(rpy[1], Vector3d::UnitY())
        *AngleAxisd(rpy[0], Vector3d::UnitX());
    return;
}

void generateBoundedR(Eigen::Matrix3d& R, double bound)
{
    Eigen::Vector3d rpy;
    rpy(0) = bound * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    rpy(1) = bound * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    rpy(2) = bound * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);

    R =  AngleAxisd(rpy[2], Vector3d::UnitZ())
        *AngleAxisd(rpy[1], Vector3d::UnitY())
        *AngleAxisd(rpy[0], Vector3d::UnitX());
    return;
}

void addNoise(Eigen::Vector3d& v_noisy, Eigen::Vector3d& v_clean, double focal_length, double pixel_noise)
{
    v_clean.normalize();

    // find good vector in normal plane based on good conditioning
    Eigen::Vector3d inPlaneVector1, inPlaneVector2;
    inPlaneVector1.setZero();

    int idx_max = 0;
    double max_value = v_clean(0);
    if (v_clean(1)>max_value){
        idx_max = 1;
        max_value = v_clean(1);
    }
    if (v_clean(2)>max_value){
        idx_max = 2;
        max_value = v_clean(2);
    }

    if (idx_max == 0){
        inPlaneVector1 << -v_clean(1)/v_clean(0), 1.0, 0.0;
    }
    if (idx_max == 1) {
        inPlaneVector1 << 0.0, -v_clean(2)/v_clean(1), 1.0;
    }
    if (idx_max == 2) {
        inPlaneVector1 << 1.0, 0.0, -v_clean(0)/v_clean(2);
    }

    // normalize the in-plane vector
    inPlaneVector1.normalize();
    inPlaneVector2 = v_clean.cross(inPlaneVector1);

    double noiseX = pixel_noise * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    double noiseY = pixel_noise * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);

    v_noisy = focal_length * v_clean + noiseX * inPlaneVector1 + noiseY * inPlaneVector2;
    v_noisy.normalize();

    return;
}

void generateRandomPoint(Eigen::Vector3d& point, double minDepth, double maxDepth)
{
    double deltaDepth = maxDepth - minDepth;

    double x = 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    double y = 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    double z = 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);

    double norm = sqrt(x*x + y*y +z*z);
    double dir_x = x/norm;
    double dir_y = y/norm;
    double dir_z = z/norm;

    double px = deltaDepth * x + minDepth * dir_x;
    double py = deltaDepth * y + minDepth * dir_y;
    double pz = deltaDepth * z + minDepth * dir_z;

    point << px, py, pz;

    return;
}

void create2D2DExperiment(int pt_number, int cam_number, double noise, double outlier_fraction, 
        std::vector<Eigen::Matrix<double, 6, 1>>& v1, std::vector<Eigen::Matrix<double, 6, 1>>& v2, 
        Eigen::Matrix3d& R_gt, Eigen::Vector3d& T_gt, double focal_length)
{
    v1.clear();
    v2.clear();

    double avg_cam_distance = 0.5;
    double minDepth = 4.0;
    double maxDepth = 8.0;
    double max_parallax = 2.0;
    double max_rotation = 0.5;


    if (pt_number<=0 || cam_number<=0 || noise<0 || outlier_fraction<0 || outlier_fraction>1)
    {
        std::cerr << "input parameters have errors!" << std::endl;
        return;
    }

    // generate the camera system
    Eigen::Vector3d cam_offset;
    std::vector<Eigen::Vector3d> cam_offsets;

    if (cam_number==1)
    {
        Eigen::Vector3d cam_offset;
        cam_offset.setZero();
        cam_offsets.push_back(cam_offset);
    }
    else
    {
        Eigen::Vector3d unit_x;
        Eigen::Matrix3d R;
        unit_x << 1.0, 0.0, 0.0;
        for (int i = 0; i < cam_number; i++) {
            generateRandomR(R);
            Eigen::Vector3d cam_offset = avg_cam_distance * R * unit_x;
            cam_offsets.push_back(cam_offset);
        }
    }

    // generate random view-points
    Eigen::Vector3d position1, position2;
    Eigen::Matrix3d rotation1, rotation2;

    position1.setZero();
    rotation1.setIdentity();

    for(int i = 0; i < 3; i++) {
        position2(i) = max_parallax * 2.0 * ( ((double) rand() / (RAND_MAX)) - 0.5);
    }
    generateBoundedR(rotation2, max_rotation);

    // compute relative translation and rotation
    R_gt = rotation1.transpose() * rotation2;
    T_gt = rotation1.transpose() * (position2 - position1);

    // Generate random point-cloud
    std::vector<Eigen::Vector3d> points;
    for(int i = 0; i < pt_number; i++) {
        Eigen::Vector3d tmp_point;
        generateRandomPoint(tmp_point, minDepth, maxDepth);
        points.push_back(tmp_point);
    }

    // Now create the correspondences by looping through the cameras
    int cam_correspondence = 0;
    std::vector<int> cam_correspondences;
    for(int i = 0; i < pt_number; i++)
    {
        cam_offset = cam_offsets[cam_correspondence];

        Eigen::Vector3d body_point1, body_point2;
        body_point1 = rotation1.transpose() * (points[i] - position1);
        body_point2 = rotation2.transpose() * (points[i] - position2);

        // we actually omit the cam rotation here by unrotating the bearing vectors already
        Eigen::Vector3d bearingVector1 = body_point1 - cam_offset;
        Eigen::Vector3d bearingVector2 = body_point2 - cam_offset;
        bearingVector1.normalize();
        bearingVector2.normalize();

        // add noise to the bearing vectors here
        Eigen::Vector3d bearingVector1_noisy, bearingVector2_noisy;
        addNoise(bearingVector1_noisy, bearingVector1, focal_length, noise);
        addNoise(bearingVector2_noisy, bearingVector2, focal_length, noise);

        Eigen::Matrix<double, 6, 1> tmp1, tmp2;
        tmp1 << bearingVector1_noisy, cam_offset;
        tmp2 << bearingVector2_noisy, cam_offset;

        v1.push_back(tmp1);
        v2.push_back(tmp2);

        // change the camera correspondence
        cam_correspondences.push_back(cam_correspondence);
        cam_correspondence++;
        if (cam_correspondence >= cam_number){
            cam_correspondence = 0;
        }
    }

    // Add outliers
    int number_outliers = static_cast<int>(outlier_fraction*pt_number);
    if (number_outliers > 0)
    {
        for(int i = 0; i < number_outliers; i++)
        {
            cam_correspondence = cam_correspondences[i];
            cam_offset = cam_offsets[i];

            // generate random point
            Eigen::Vector3d point, body_point2, bearingVector2;
            generateRandomPoint(point, minDepth, maxDepth);
            body_point2 = rotation2.transpose() * (point - position2);
            // store the point (no need to add noise)
            bearingVector2 = body_point2 - cam_offset;
            // store the normalized bearing vectors along with the cameras they are
            // being seen (we create correspondences that always originate from the
            // same camera, you can change this if you want)
            bearingVector2.normalize();

            Eigen::Matrix<double, 6, 1> tmp2;
            tmp2 << bearingVector2, cam_offset;
            v2[i] = tmp2;
        }
    }
    
    return;
}

void create2D2DExperiment(int pt_number, double noise, double outlier_fraction, 
        std::vector<Eigen::Matrix<double, 3, 1>>& v1, std::vector<Eigen::Matrix<double, 3, 1>>& v2, 
        Eigen::Matrix3d& R, Eigen::Vector3d& t, double focal_length)
{
    std::vector<Eigen::Matrix<double, 6, 1>> gv1, gv2;
    create2D2DExperiment(pt_number, 1, noise, outlier_fraction, gv1, gv2, R, t);

    for(unsigned int i = 0; i < gv1.size(); i++)
    {
        v1.push_back(gv1[i].block(0,0,3,1));
        v2.push_back(gv2[i].block(0,0,3,1));
    }

    return;
}
