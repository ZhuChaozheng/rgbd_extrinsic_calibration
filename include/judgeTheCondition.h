
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <algorithm>
#include "getCameraOrientation.h"
#include "pointCloudTransform.h"

bool judgeTheCondition(Eigen::Isometry3d transfrom, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_plane,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain);
