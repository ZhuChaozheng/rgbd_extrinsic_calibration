//
// Created by jason on 12/8/19.
//

#ifndef RGBDEXTRINSICCALIBRATION_ESTIMATEPLANE_H
#define RGBDEXTRINSICCALIBRATION_ESTIMATEPLANE_H

#include <iostream>
#include <thread>
#include <string>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

/*
* these values will be returned as extern 
*/
extern pcl::ModelCoefficients::Ptr coefficients;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_plane;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain;

void estimatePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
#endif //RGBDEXTRINSICCALIBRATION_ESTIMATEPLANE_H
