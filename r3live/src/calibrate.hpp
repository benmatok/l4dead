#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#pragma once
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
#include <kd_tree/ikd_Tree.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3.h>
#include <FOV_Checker/FOV_Checker.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "lib_sophus/so3.hpp"
#include "lib_sophus/se3.hpp"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_thread_pool.hpp"
#include "tools_ros.hpp"

#include "loam/IMU_Processing.hpp"

#include "image_frame.hpp"
#include "pointcloud_rgbd.hpp"
#include "rgbmap_tracker.hpp"




class Calibrate
{
  public:
     ros::NodeHandle  m_ros_node_handle;
     std::vector< cv::Vec3d> rot_camera_time ;
     std::vector< cv::Vec3d> trans_image_time ; 
     std::vector< double>  pose_camera_time ;
     ros::Subscriber sub_img ;  
     std::vector<std::vector<cv::Point2f>> allCharucoCorners;
     std::vector<std::vector<int>> allCharucoIds;
     cv::Ptr< cv::aruco::Dictionary > dict;
     cv::Ptr< cv::aruco::CharucoBoard > charuco; 
     int frame_number; 
    void img_cbk(const sensor_msgs::ImageConstPtr &msg);
    void detectCharucoBoardWithoutCalibration(cv::Mat image ,  std::vector<cv::Point2f> &charucoCorners , std::vector<int> &charucoIds) ;
    void detectCharucoBoardWithCalibrationPose(cv::Mat &image ) ;
    void calibrate_camera_in(cv::Mat &image) ; 






    Calibrate()
    {
        frame_number = 0 ;
        dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        charuco  = cv::aruco::CharucoBoard::create(7, 5,0.04, 0.04/2, dict);
        sub_img = m_ros_node_handle.subscribe("/camera/color/image_raw", 1000000, &Calibrate::img_cbk, this, ros::TransportHints().tcpNoDelay());

    }

};
