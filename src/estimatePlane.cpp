//
// Created by jason on 12/7/19.
//

/*
 * return model, inlier and outlier
 *
 * return inliers, model_p
 *
 * remainPointCloud = outlier
 */
#include "estimatePlane.h"

// construct a object of model parameter to record result
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain (new pcl::PointCloud<pcl::PointXYZ>);

/*
 * estimatePlane()
 * @pointcloud
 * return model(coefficients), point_cloud_plane(in camera coordinate),
 *        remain_point_cloud(in camera coordinate)
 *
 */
void estimatePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {

    // initialize PointClouds

    // inliers expresses the point can be ignored in errors, to record its order
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // construct a segmentor
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory- set target geometry shape
    seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setModelType (pcl::SACMODEL_LINE);

    // segmetation method: ransac
    seg.setMethodType (pcl::SAC_RANSAC);

    // the range of tolerance errors
    seg.setDistanceThreshold (0.005);

    // input pointcloud data
    seg.setInputCloud (point_cloud);

    // segment pointcloud
    seg.segment (*inliers, *coefficients);

    // display the coefficients of model
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;


    // Extract the planar inliers from the input cloud
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(point_cloud);
    // extract.setIndices(inliers);
    // extract.setNegative (false);
    // extract.filter (*point_cloud_plane);
    
    // if (inliers->indices.size() > 0)
       // pcl::io::savePCDFile("./pointcloud_new.pcd", *point_cloud_plane);
    // std::cerr << "point_cloud: " << point_cloud->points.size () << std::endl;
    // for (size_t i = 0; i < 50; ++i)
    //     std::cerr << "inliers: \n " << point_cloud->points[inliers->indices[i]].x << " "
    //               << point_cloud->points[inliers->indices[i]].y << " "
    //               << point_cloud->points[inliers->indices[i]].z << std::endl;
    
    // std::cout << "point_cloud_plane:\n" << point_cloud_plane->points.size() << std::endl;
    

    /*
     * remain the data except plane, outliers
     * 
     * if you want eliminate other planes, please uncomment the following sentence
     * 
     */
    // extract.setNegative(true);
    // extract.filter(*point_cloud_remain);

    


        
}
