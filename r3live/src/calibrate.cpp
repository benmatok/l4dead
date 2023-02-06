#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>



void start_calibration(Calibrate * calibrate_instance)
{

    while(calibrate_instance->image_queue.size() < 500 )
    {
      


    }
    calibrate_instance->sub_img.shutdown();
    calibrate_instance->sub_imu.shutdown();
    std::vector<double> time_of_images ; 
    std::vector<std::tuple<cv::Vec3d,cv::Vec3d>> trajectory_camera ;




    while(calibrate_instance->image_queue.size() > 0)
    {


    sensor_msgs::ImageConstPtr msg  = calibrate_instance->image_queue.pop() ; 
    cv::Vec3d  rvec ;
    cv::Vec3d tvec ; 
    cv::Mat image = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    calibrate_instance->detectCharucoBoardWithCalibrationPose(image ,rvec , tvec) ;
    if (rvec[0] != 0 || rvec[1] !=0 || rvec[2]!=0  || tvec[0] != 0 ||  tvec[1] !=0 || tvec[2] !=0)
    { 
         std::tuple<cv::Vec3d,cv::Vec3d> state = std::make_tuple(rvec , tvec);
         trajectory_camera.push_back(state); 
         time_of_images.push_back(calibrate_instance->header.stamp) ; 


    }

    }






}




int main(int argc, char **argv)
{

    Eigen::initParallel();
    ros::init(argc, argv, "calibrate");
    Calibrate * calibrate_instance = new Calibrate();
    std::thread thread_obj(start_calibration, calibrate_instance);
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::spin();




}