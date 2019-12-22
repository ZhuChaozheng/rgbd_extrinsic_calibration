// created by Jason, 06 Dec, 2019.
#include "generatePointCloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(Mat depth,...) {
    // point cloud variable
    // intelligent point will construct a null point cloud, and free its space after that
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // loop depth image
    for (int m = 0; m < depth.rows; m++){
      for (int n = 0; n < depth.cols; n++){
          // // get the value of (m, n) in the depth imgae
          float d = depth.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
          // if d does not exist, overlook this point
          // cout << "d\n" << d << endl;
          if (d == 0)
             continue;
          // if d exist, then add a new point into point cloud
          pcl::PointXYZRGB p;

          // calculate this point's space coordinate
          p.z = double(d) / camera_factor;
          p.x = (n - camera_cx) * p.z / camera_fx;
          p.y = (m - camera_cy) * p.z / camera_fy;

          // rgb's real sequence is BGR
          // p.b = rgb.ptr<uchar>(m)[n*3];
          // p.g = rgb.ptr<uchar>(m)[n*3+1];
          // p.r = rgb.ptr<uchar>(m)[n*3+2];
          p.b = 100;
          p.g = 130;
          p.r = 100;

          // add p into point cloud
          cloud->points.push_back( p );
      }
    }


    // save point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();

    cloud->is_dense = false;

    // pcl::io::savePCDFile("./pointcloud_new_1.pcd", *cloud);
    // cout << "cloud = \n" << cloud->points.size() << endl;

    // convert xyzrgb to xyz
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud->points.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); ++i) {
        point_cloud->points[i].x = cloud->points[i].x;
        point_cloud->points[i].y = cloud->points[i].y;
        point_cloud->points[i].z = cloud->points[i].z;
    }
    // save point cloud
    point_cloud->height = 1;
    point_cloud->width = point_cloud->points.size();

    point_cloud->is_dense = false;

    // cout << "point_cloud = \n" << point_cloud->points.size() << endl;
    // pcl::io::savePCDFile("./pointcloud_new_1.pcd", *point_cloud);


    // clear data and exit
    cloud->points.clear();
  //  point_cloud->points.clear();



    return point_cloud;

}
