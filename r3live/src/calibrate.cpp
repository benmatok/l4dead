#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>
#include <fstream>




void Calibrate::copyvec(cv::Vec3d &copyto , cv::Vec3d &empty)
{
    empty[0] = copyto[0] ; 
    empty[1] = copyto[1] ; 
    empty[2] = copyto[2] ; 



}










void start_calibration(Calibrate * calibrate_instance)
{


    int flip =  std::round(4000/6) ;
    while(calibrate_instance->image_queue.size() < 4000 )
    {
     int conc_size = calibrate_instance->image_queue.size() ; 
      if(conc_size % flip == 0  && conc_size!=0  )
      {
        std::cout <<  "flip" << conc_size << std::endl;

      }


    }
    calibrate_instance->sub_img.shutdown();
    calibrate_instance->sub_imu.shutdown();
    calibrate_instance->save_camera_frames();
    calibrate_instance->find_static_time();
    if(calibrate_instance->time_diff_static>0,8) 
    {
        calibrate_instance->save_imu() ; 
        //calibrate_instance->calc_gyro_bias() ; 

    }
    calibrate_instance->calc_accel_bias();





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
