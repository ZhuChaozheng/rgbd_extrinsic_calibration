#include <iostream>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                           Eigen::Affine3d &transfrom);


