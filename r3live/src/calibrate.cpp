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



cv::Vec3d to_vec3d(cv::Mat1f const& m)
{
    return cv::Vec3d(m.at<float>(0), m.at<float>(1), m.at<float>(2));
}


void Calibrate::calc_extrinsic()
{   

    std::vector<std::tuple<double,double>> intervals;
    Calibrate::find_interval(intervals  );

    for(int i = 0; i<intervals.size() ; i++)
    {
        cv::Vec3d imu_translation ;
        imu_translation[0] = 0; 
        imu_translation[1] = 0;
        imu_translation[2] = 0;  

        cv::Mat imu_rotation =  (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1); 
        cv::Vec3d imu_velocity  ;
        imu_velocity[0] = 0;
        imu_velocity[1] = 0;
        imu_velocity[2] = 0;

        double c1_time = std::get<1>(intervals[i]) ; 
        auto upper1 = std::upper_bound(time_imu.begin(), time_imu.end(), c1_time);
        int imu_index_i1 = upper1 -time_imu.begin();
        double dt_imu =  time_imu[1] -  time_imu[0] ; 
        cv::Vec3d accel_i1 = imu_accels[imu_index_i1];

        //cv::Vec3d accel_i2 = imu_accels[imu_index_i2];
        if(imu_accels.size() - imu_index_i1>=50)
        {


            for(int j =imu_index_i1 +1 ; j<imu_accels.size() ; j++)
            {
                cv::Vec3d curr_angular_velocity = angular_velocities[j] ; 
                 cv::Vec3d curr_rotation ; 
                cv::Rodrigues(curr_angular_velocity , curr_rotation );
                imu_rotation = curr_rotation *  imu_rotation ;
                cv::Vec3d  curr_accel = imu_accels[j] ; 
                cv::Vec3d movement = (0.5*dt_imu*dt_imu)*to_vec3d(imu_rotation*curr_accel) ; 
                imu_translation = imu_translation + dt_imu*imu_velocity  ; 
                imu_velocity = imu_velocity*dt_imu +  to_vec3d(imu_rotation *curr_accel)*dt_imu ; 





            }





        }
        
        //cv::Vec3d angular_velocity_i1 = angular_velocities[imu_index_i1] ; 

        //cv::Vec3d angular_velocity_i2 = angular_velocities[imu_index_i2] ; 
        //double delta_time = time_imu[imu_index_i2] - time_imu[imu_index_i1]
        //cv::Mat rotation_i1_to_i2;
        //cv::Rodrigues(angular_velocity_i2 , rotation_i1_to_i2 );


        //cv::Vec3d state_i2 =  (rotation_i1_to_i2 )* accel_i2 * delta_time






        




    }

}





void start_calibration(Calibrate * calibrate_instance)
{


    //int flip =  std::round(4000/7) ;
    while(calibrate_instance->image_queue.size() < 3000 )
    {
     //int conc_size = calibrate_instance->image_queue.size() ; 
      //if(conc_size % flip == 0  && conc_size!=0  )
      //{
        //std::cout <<  "flip" << conc_size << std::endl;

      //}


    }
    calibrate_instance->sub_img.shutdown();
    calibrate_instance->sub_imu.shutdown();
    calibrate_instance->save_camera_frames();
    calibrate_instance->find_static_time();
    calibrate_instance->save_imu() ; 
    calibrate_instance->calc_gyro_bias() ; 
    calibrate_instance->calc_extrinsic();
    //if(calibrate_instance->time_diff_static>0.8) 
    //{
        //calibrate_instance->save_imu() ; 
        //calibrate_instance->calc_gyro_bias() ; 

    //}
    //calibrate_instance->calc_accel_bias();





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
