#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>


class Calibrate
{
  public:
     ros::NodeHandle  m_ros_node_handle;
     std::vector< cv::Vec3d> rot_camera_time ;
     std::vector< cv::Vec3d> trans_image_time ; 
     std::vector< double>  pose_camera_time ;
     ros::Subscriber sub_img
    


    
    Calibrate()
    {
        sub_img = m_ros_node_handle.subscribe(IMAGE_topic.c_str(), 1000000, &Calibrate::image_callback, this, ros::TransportHints().tcpNoDelay());

    }

}
