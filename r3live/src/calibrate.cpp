#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>




void copyvec(cv::Vec3d &copyto , cv::Vec3d &empty)
{
    empty[0] = copyto[0] ; 
    empty[1] = copyto[1] ; 
    empty[2] = copyto[2] ; 



}

void start_calibration(Calibrate * calibrate_instance)
{

    while(calibrate_instance->image_queue.size() < 500 )
    {
      


    }
    calibrate_instance->sub_img.shutdown();
    calibrate_instance->sub_imu.shutdown();
    std::vector<double> time_of_images ; 
    std::vector<std::tuple<cv::Vec3d,cv::Vec3d>> trajectory_camera ;
    std::vector<bool> is_still ;
    is_still.push_back(true) ; 
    bool start = true ; 
    cv::Vec3d  last_rvec ;
    cv::Vec3d last_tvec ;  
    
    while(calibrate_instance->image_queue.size() > 0)
    {


    sensor_msgs::ImageConstPtr msg  ; 
    calibrate_instance->image_queue.pop(msg) ; 

    cv::Vec3d  rvec ;
    cv::Vec3d tvec ; 
    cv::Mat image = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    calibrate_instance->detectCharucoBoardWithCalibrationPose(image ,rvec , tvec) ;
    if (rvec[0] != 0 || rvec[1] !=0 || rvec[2]!=0  || tvec[0] != 0 ||  tvec[1] !=0 || tvec[2] !=0)
    { 
        if (start)
        {
            copyvec(last_rvec , rvec) ;
            copyvec(last_tvec , tvec) ; 
            start = false;

         
        }
        else
        {
            cv::Vec3d  tvec_movement = tvec -last_tvec ;  
            cv::Vec3d  rvec_movement = rvec -last_rvec ;
            if(tvec_movement[0] < 0.01 && tvec_movement[1] < 0.01 && tvec_movement[2] < 0.01 && rvec_movement[0] < 0.01 && rvec_movement[1] < 0.01 && rvec_movement[2] < 0.01) 
            {
                is_still.push_back(true);

                
            } 

            else
            {
                is_still.push_back(false)  ;
            }




        }
         std::tuple<cv::Vec3d,cv::Vec3d> state = std::make_tuple(rvec , tvec);
         trajectory_camera.push_back(state); 
          std_msgs::Header h = msg->header;
         double time =  double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
         time_of_images.push_back(time); 

         copyvec(last_rvec , rvec) ;
         copyvec(last_tvec , tvec) ; 




    }

    }


std::cout<< "finished" << std::endl;
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