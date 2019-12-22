/*
 * this function is used as display by plot in real-time, but now, in order to
 * reduce the complexity of program, we are going to transfer the data to other
 * language platform.
 * @para:
 *        pc: raw point cloud
 *        model: coefficients
 *        location: tform.T(4, 1:3)
 *        orientation: tform.T(1:3, 1:3)
 *
 * output: location, orientation, pc, model,
 *         pc_new: all point cloud converted from pc to pc_new through transform(tform)
 */
#include "showDepthSensorModel.h"
#include "judgeTheCondition.h"

void showDepthSensorModel(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                          pcl::ModelCoefficients::Ptr coefficients, Eigen::Isometry3d transform,
                          Eigen::Matrix3d R, Eigen::Vector3d t) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_new (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::transformPointCloud (*point_cloud, *point_cloud_new, transform);
    // std::cout << "After Model coefficients: " << coefficients->values[0] << " "
    //           << coefficients->values[1] << " "
    //           << coefficients->values[2] << " "
    //           << coefficients->values[3] << std::endl;
    // std::cout << "R\n" << R << std::endl;
    // std::cout << "t\n" << t << std::endl;

}
