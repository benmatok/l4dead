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
#include "tbb/concurrent_queue.h"
#include<tuple>
#include <thread>
#include <Eigen/Dense>
class Calibrate
{
  public:
     ros::NodeHandle  m_ros_node_handle;
     ros::Subscriber sub_img ;  
     ros::Subscriber sub_imu ;  
     std::vector<std::vector<cv::Point2f>> allCharucoCorners;
     std::vector<std::vector<int>> allCharucoIds;
     cv::Ptr< cv::aruco::Dictionary > dict;
     cv::Ptr< cv::aruco::CharucoBoard > charuco; 
     int frame_number; 
     cv::Mat cameraMatrix ;
     cv::Mat distCoeffs;
     tbb::concurrent_bounded_queue<sensor_msgs::ImageConstPtr > image_queue ; 
     tbb::concurrent_bounded_queue<sensor_msgs::Imu::ConstPtr > imu_queue ; 

     std::vector<double> time_of_images;
     std::vector<std::tuple<cv::Vec3d,cv::Vec3d>>  trajectory_camera;
     std::vector<bool> is_still; 



     std::vector<double> time_imu;
     std::vector<cv::Vec3d> angular_velocities ; 
      std::vector<cv::Vec3d> imu_accels ; 
     cv::Vec3d gyro_bias;  



     double start_time_static ; 
     double end_time_static; 
     double time_diff_static ;
     
    
    void img_cbk(const sensor_msgs::ImageConstPtr &msg);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg);
    void detectCharucoBoardWithoutCalibration(cv::Mat image ,  std::vector<cv::Point2f> &charucoCorners , std::vector<int> &charucoIds) ;
    void detectCharucoBoardWithCalibrationPose(cv::Mat &image , cv::Vec3d &rvec , cv::Vec3d &tvec) ;
    void calibrate_camera_in(cv::Mat &image) ; 
    void img_cbk_calibrate(const sensor_msgs::ImageConstPtr &msg);
    void save_camera_frames();
    void find_static_time();
    void copyvec(cv::Vec3d &copyto , cv::Vec3d &empty);
    void save_imu() ; 
    void calc_gyro_bias() ; 
    void calc_accel_bias();
    void sphere_fit(std::vector<cv::Vec3d> &points_fit , Eigen::VectorXd &c );






    Calibrate()
    {

        frame_number = 0 ;
        dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        charuco  = cv::aruco::CharucoBoard::create(4, 4,0.0485, 0.0485/2, dict);

        //double distCoeffs_array[5]  = {-0.05224373092319542, 0.0609873425248403, 0.001433938108933573, -0.0002926947407053129, -0.01567957391005009};
        //double cameraMatrix_array[3][3] =  {{636.4715066042678, 0, 632.8146215973151} ,{0, 637.3852410290077, 380.4818147863101} , {0, 0, 1}};


        //distCoeffs = cv::Mat(5, 1, CV_64F, distCoeffs_array);
        distCoeffs  = (cv::Mat_<double>(1,5) <<-0.05224373092319542, 0.0609873425248403, 0.001433938108933573, -0.0002926947407053129, -0.01567957391005009);
        cameraMatrix = (cv::Mat_<double>(3,3) <<636.4715066042678, 0, 632.8146215973151 ,0, 637.3852410290077, 380.4818147863101  , 0, 0, 1); 
        //cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrix_array);
        std::cout << distCoeffs << std::endl;
        sub_img = m_ros_node_handle.subscribe("/camera/color/image_raw", 1000000, &Calibrate::img_cbk, this, ros::TransportHints().tcpNoDelay());
        sub_imu = m_ros_node_handle.subscribe("/camera/imu", 1000000, &Calibrate::imu_cbk, this, ros::TransportHints().tcpNoDelay());

    }

};
