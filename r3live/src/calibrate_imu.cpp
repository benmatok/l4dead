#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>
#include "tools_mem_used.h"
#include "tools_logger.hpp"
#include <bits/stdc++.h>



void  Calibrate::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg)
{


    //std::cout << imu_msg_raw->angular_velocity[0] << std::endl;
    imu_queue.push(msg);
    


}






void Calibrate::find_static_time()
{

     start_time_static = 0 ; 
     end_time_static =  0 ; 
     time_diff_static = 0  ;
     



    bool before_true = true;
    int start_index = 0 ;
        for(int i =1 ; i< is_still.size() ; i++)
    {
        if(is_still[i])
        {
            if(!before_true)
            {
                start_index = i;
            }
            before_true = true; 

        }
        else{

            
            if (before_true)
            {
                double is_max_time = time_of_images[i-1] - time_of_images[start_index] ; 
                if(is_max_time > time_diff_static )
                {
                    start_time_static = time_of_images[start_index] ;
                    end_time_static = time_of_images[i-1] ; 
                    time_diff_static  = is_max_time ;  

                }

            }
            before_true = false; 
        }


    }
    if(start_index ==0 && before_true )
    {
        time_diff_static  =  time_of_images[is_still.size()-1] - time_of_images[start_index] ; 
        start_time_static = time_of_images[0] ;
        end_time_static = time_of_images[is_still.size()-1] ; 


    }
    std::cout << "time static " << time_diff_static << std::endl;

}



void  Calibrate::save_imu(){
    while(imu_queue.size() > 0)
    {
        sensor_msgs::Imu::ConstPtr msg  ; 
         imu_queue.pop(msg) ; 
        
        std_msgs::Header h = msg->header;
        double time =  double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;
        time_imu.push_back(time) ; 
        cv::Vec3d angular_velocity ; 
        angular_velocity[0] = msg->angular_velocity.x ; 
        angular_velocity[1] = msg->angular_velocity.y ;
        angular_velocity[2] = msg->angular_velocity.z ;
        angular_velocities.push_back(angular_velocity);

        cv::Vec3d accel ; 
        accel[0] = msg->linear_acceleration.x ; 

        accel[1] =  msg->linear_acceleration.y ;
        accel[2] =  msg->linear_acceleration.z ;
        imu_accels.push_back(accel) ; 
         
         
         
         
         
         

    }

}






void  Calibrate::calc_gyro_bias(){
    double counter = 0 ; 
    cv::Vec3d sum ;
    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0; 
    for(int i = 0;i <time_imu.size() ; i++)
    {
        if(time_imu[i] > start_time_static && time_imu[i] < end_time_static )
        {
            sum=sum+angular_velocities[i] ; 
            counter+=1 ; 
        }
    }
    gyro_bias = sum*(1/counter)  ;
    std::cout << gyro_bias << std::endl;

    
}




void Calibrate::sphere_fit(std::vector<cv::Vec3d> &points_fit , Eigen::VectorXd &c )
{
    Eigen::MatrixXd A(points_fit.size(),4);
    Eigen::VectorXd f(points_fit.size());
    for(int i = 0 ;i <points_fit.size() ; i++)
    {
        A(i, 0 ) = 2*points_fit[i][0];
        A(i, 1 ) = 2*points_fit[i][1]; 
        A(i, 2 ) = 2*points_fit[i][2]; 
        A(i, 3 ) = 1;
        f[i] = points_fit[i][0]* points_fit[i][0] ;
        f[i]+=points_fit[i][1]* points_fit[i][1];
        f[i]+=points_fit[i][2]* points_fit[i][2];
    }
    c = A.colPivHouseholderQr().solve(f);
}




void  Calibrate::calc_accel_bias(){

    std::ofstream MyFile("/code/accels.txt");
    std::vector<cv::Vec3d> no_movement_frames ;
    for(int i = 0;i<is_still.size() ; i++ )
    {
        if(is_still[i])
        {
            auto upper1 = std::upper_bound(time_imu.begin(), time_imu.end(), time_of_images[i]);

            int time_index = upper1 -time_imu.begin();
            cv::Vec3d accel = imu_accels[time_index];
            no_movement_frames.push_back(accel);
            MyFile <<std::setprecision(64) <<  accel[0] << " " << accel[1] << " " << accel[2] << std::endl;



        }
        



    }






    MyFile.close();





    Eigen::VectorXd c;

    sphere_fit(no_movement_frames , c);
    std::cout << "c: " <<c  << std::endl;
    double radius = std::sqrt(c[3] + c[0]*c[0] + c[1]*c[1] +c[2]*c[2] ); 

     std::cout << "radius " <<radius  << std::endl;
     







    







}