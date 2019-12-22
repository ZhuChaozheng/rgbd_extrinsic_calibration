#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

void showDepthSensorModel(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                          pcl::ModelCoefficients::Ptr coefficients, Eigen::Isometry3d transform,
                          Eigen::Matrix3d R, Eigen::Vector3d t);
