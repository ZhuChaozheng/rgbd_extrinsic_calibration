/*
 * created by Jason. 10 Dec. 2019.
 * this function is used as a node in ROS.
 * mainly function is:
 * 1. receive depth image
 * 2. call key functions
 */


// add ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

// add key functions libraries
#include "estimatePlane.h"
#include "generatePointCloud.h"
#include "getCameraOrientation.h"
#include "getExternalParameterFromPlane.h"
#include "judgeTheCondition.h"
#include "pointCloudTransform.h"
#include "decomposeMatrixT.h"
#include "showDepthSensorModel.h"

using namespace pcl;
using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "rgbdextrinsiccalibration";

class RGBDExtrinsicCalibration {
    // add ROS image dependencies
    ros::NodeHandle nh_;
    ros::Publisher string_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_depth_sub_;
    image_transport::Subscriber image_rgb_sub_;



    bool is_continue = true;
    double delta = 0.01;

public:
    cv_bridge::CvImagePtr depth_ptr;
    cv::Mat depth_pic;

    cv_bridge::CvImagePtr rgb_ptr;
    cv::Mat rgb_pic;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    int counter = 0;


public:
  RGBDExtrinsicCalibration() : it_(nh_) {

    // Subscribe to input video feed and publish output video feed
    image_rgb_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                 &RGBDExtrinsicCalibration::rgbCallback, this);

    // Subscribe to input video feed and publish output video feed
    image_depth_sub_ = it_.subscribe("/camera/depth/image", 1,
                 &RGBDExtrinsicCalibration::depthCallback, this);

    // publish string message to topic "/pixel_distance/steering"
    // string_pub_ = nh_.advertise<std_msgs::String>("/pixel_distance/steering", 1);

    // cv::namedWindow(OPENCV_WINDOW);

}

~RGBDExtrinsicCalibration() {
    // cv::destroyWindow(OPENCV_WINDOW);
}

void rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
    // try
    // {
    //   //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
    //   // depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    //   cv::imshow(OPENCV_WINDOW, cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
    //   rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    //   cv::waitKey(200);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("Could not convert from '%s' to 'mono16'.", rgb_msg->encoding.c_str());
    // }

    // rgb_pic = rgb_ptr->image;

}
void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    //cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

          cv::waitKey(200);
    }
    catch (cv_bridge::Exception& e)
    {   
         ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }

    depth_pic = depth_ptr->image;
    // cv::imwrite("hello.png", depth_pic);
    counter ++;
    cout << "counter: " << counter << endl;

    // output some info about the depth image in cv format
    // cout<<"output some info about the depth image in cv format"<<endl;
    // cout<<"rows of the depth image = "<<depth_pic.rows<<endl;
    // cout<<"cols of the depth image = "<<depth_pic.cols<<endl;
    // cout<<"type of depth_pic's element = "<<depth_pic.type()<<endl;

    // call key functions

    extrinsicCalibration(depth_pic);
    Eigen::Vector3d euler_angles = R.eulerAngles ( 2,1,0 );
    // cout << "yaw pitch roll = " << euler_angles.transpose() << endl;


    transform.setOrigin( tf::Vector3(t(0), t(1), t(2)) );
    tf::Quaternion q;
    q.setRPY(euler_angles(2), euler_angles(1), euler_angles(0));
    transform.setRotation(q);
    cout << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "world_link"));

}

void extrinsicCalibration(Mat depth,...) {
    is_continue = true;
    /*
     * generatePointCloud
     * @para:path
     * return pcl::PointCloud<pcl::PointXYZ>:Ptr cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    point_cloud = generatePointCloud(depth);

    while(is_continue) {

        /*
         * estimatePlane()
         * @para: pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
         * return extern model(coefficients) point_cloud_plane
         *        and point_cloud_remain(pcl::PointCloud<pcl::PointXYZ>::Ptr)
         *
         */
        estimatePlane(point_cloud);

        // if the number of remain points is too few to continue
        // if (point_cloud_remain->points.size() < point_cloud_plane->points.size() * delta ) {
        //     break;
        // }

        /*
         * getExternalParameterFromPlane
         * @para: coefficients
         * return Isometry3d (Transformation matrix)
         */
        Eigen::Isometry3d transform;
        transform = getExternalParameterFromPlane(coefficients);
        

        /*
        * bool judgeTheCondition()
        * @para: Eigen::Isometry3d transfrom, pcl::PointCloud<pcl::PointXYZ>::Ptr
        *        point_cloud_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_remain
        * return is_continue (bool)
        *
        * judge condition (az, pc_world, pc_plane)
        * az < pi/4 or median(pc_plane) > min(pc_world) + \e
        */
        // it will obtain R, t
        decomposeMatrixT(transform);
        cout << "R: \n " << R << endl;
        cout << "t: \n " << t << endl;
        // if it enables the judgement, please comment the last code
        //is_continue = judgeTheCondition(transform, point_cloud_plane, point_cloud_remain);
        is_continue = false;

        // output results from decomposeMatrixT()
        /*
        * void showDepthSensorModel(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        *                           pcl::ModelCoefficients::Ptr coefficients, Eigen::Isometry3d transform,
        *                           Eigen::Matrix3d R, Eigen::Vector3d t)
        * output: cout all the results
        */
        // showDepthSensorModel(point_cloud, coefficients, transform, R, t);

        // use the remain point to continue refining
        // point_cloud = point_cloud_remain;
    }
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgbdextrinsiccalibration");
    RGBDExtrinsicCalibration rgbdextrinsiccalibration;

    while(ros::ok()) {

        // try {
        //     // cout << "second\n" << rgbdextrinsiccalibration.depth_pic.rows << endl;
        //     rgbdextrinsiccalibration.extrinsicCalibration(rgbdextrinsiccalibration.depth_pic);
        //
        // catch (cv_bridge::Exception& e) {
        //     ROS_ERROR("data could not receive");
        // }

        ros::spin();
    }

    return 0;
}
