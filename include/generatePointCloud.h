//
// Created by jason on 12/8/19.
//

#ifndef RGBDEXTRINSICCALIBRATION_GENERATEPOINTCLOUD_H
#define RGBDEXTRINSICCALIBRATION_GENERATEPOINTCLOUD_H

// C++ library
#include <iostream>
#include <string>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// camera intrinsic for spark
const double camera_factor = 1;
const double camera_cx = 314.005672;
const double camera_cy = 242.391705;
const double camera_fx = 520.377485;
const double camera_fy = 521.494097;


using namespace std;
using namespace cv;

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(Mat depth,...);

#endif //RGBDEXTRINSICCALIBRATION_GENERATEPOINTCLOUD_H
