#include <iostream>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

Eigen::Matrix3d translate2RigidMatrix(Eigen::Matrix3d R);

Eigen::Vector3d getProjectedPoint(Eigen::Vector3d v, pcl::ModelCoefficients::Ptr coefficients);

Eigen::Isometry3d getExternalParameterFromPlane (pcl::ModelCoefficients::Ptr coefficients);

