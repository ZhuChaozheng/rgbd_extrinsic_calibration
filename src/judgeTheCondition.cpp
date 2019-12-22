//
// Created by jason on 12/8/19.
//

/*
 * median P > min P, or delete the inlier
 * \theta < pi/4, or delete the inlier
 */

#include "judgeTheCondition.h"
#include "decomposeMatrixT.h"


bool judgeTheCondition(Eigen::Isometry3d transform, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_plane,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain) {
    bool is_finish = true;
    // when the variable is null, we should stop it
    if (point_cloud_plane->points.size() == 0 || point_cloud_remain->points.size() == 0) {
        is_finish = false;
        return is_finish;
    }

    // produce three shared variables, T(Matrix4d), R(Matrix3d), t(Vector3d)
    decomposeMatrixT(transform);

    double az = getCameraOrientation(R);

    if (abs(az) > (M_PI/4)) {
        is_finish = false;
        return is_finish;
    }

    //    Eigen::Isometry3d tformC2W;
    Eigen::Affine3d tformC2W;
    tformC2W = transform.inverse();

    // transform the variables of point_cloud_plane and point_cloud_remain from camera coordinate to world
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain_world (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_plane_world (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*point_cloud_plane, *point_cloud_plane_world, tformC2W);
    pcl::transformPointCloud (*point_cloud_remain, *point_cloud_remain_world, tformC2W);


    // extract the vector of z for plane
    std::vector<double> point_cloud_plane_z;
    for (size_t i = 0; i < point_cloud_plane_world->points.size(); ++i)
        point_cloud_plane_z.push_back(point_cloud_plane_world->points[i].z);

    // median for P^Z_{W_1}
    double median;
    sort(point_cloud_plane_z.begin(), point_cloud_plane_z.end());
    if (point_cloud_plane_z.size() % 2 == 0) {
         median = (point_cloud_plane_z[point_cloud_plane_z.size()/2 - 1]
                         + point_cloud_plane_z[point_cloud_plane_z.size()/2]) / 2;
    }
    else {
        median = point_cloud_plane_z[point_cloud_plane_z.size()/2];
    }
    std::cout << "median: \n" << point_cloud_plane_z[point_cloud_plane_z.size()/2] << std::endl;

    // extract the vector of z for remain
    std::vector<double> point_cloud_remain_z;
    for (size_t i = 0; i < point_cloud_remain_world->points.size(); ++i)
        point_cloud_remain_z.push_back(point_cloud_remain_world->points[i].z);
    // min_value for P^Z_{C_0}
    int min_value = *min_element(point_cloud_remain_z.begin(), point_cloud_remain_z.end());
    if (median < min_value) {
        is_finish = false;
        return is_finish;
    }

    is_finish = true;

    return is_finish;
}
