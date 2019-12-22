//
// Created by jason on 12/8/19.
//

/*
     *
     * pctransform(pointcloud, tfrom)
     * @para: pointcloud in, Transformation matrix
     * return pointcloud out
     *
     *
     */

#include "pointCloudTransform.h"
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                           Eigen::Affine3d &transfrom) {
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    std::cout << transfrom(0, 1) << std::endl;
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transfrom);

    return transformed_cloud;


}
