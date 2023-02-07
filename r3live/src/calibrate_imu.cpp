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




    std::ofstream MyFile("/code/gyro.txt");
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

        MyFile << std::setprecision(64) <<  angular_velocity[0] << " " << angular_velocity[1] << " " << angular_velocity[2] << std::endl;
        angular_velocities.push_back(angular_velocity);

        cv::Vec3d accel ; 
        accel[0] = msg->linear_acceleration.x ; 

        accel[1] =  msg->linear_acceleration.y ;
        accel[2] =  msg->linear_acceleration.z ;
        imu_accels.push_back(accel) ; 
         
         
         
         
         
         

    }
    MyFile.close();
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


void Calibrate::sphere_fit_vector(std::vector<cv::Vec3d> &points_fit , Eigen::VectorXd &c  , std::vector<int> &sphere_indecies)
{
    Eigen::MatrixXd A(sphere_indecies.size(),4);
    Eigen::VectorXd f(sphere_indecies.size());
    int counter= 0 ;
    for (int  i =0 ; i < sphere_indecies.size() ; i++ ) 
    {
        int index = sphere_indecies[i] ; 
        A(i, 0 ) = 2*points_fit[index][0];
        A(i, 1 ) = 2*points_fit[index][1]; 
        A(i, 2 ) = 2*points_fit[index][2]; 
        A(i, 3 ) = 1;
        f[i] = points_fit[index][0]* points_fit[index][0] ;
        f[i]+=points_fit[index][1]* points_fit[index][1];
        f[i]+=points_fit[index][2]* points_fit[index][2];
        counter++;
    }
    c = A.colPivHouseholderQr().solve(f);
}





void Calibrate::sphere_fit_set(std::vector<cv::Vec3d> &points_fit , Eigen::VectorXd &c  , std::set<int> &sphere_indecies)
{
    Eigen::MatrixXd A(4,4);
    Eigen::VectorXd f(4);
    int counter= 0 ;
    for (auto&  i : sphere_indecies ) 
    {

        A(counter, 0 ) = 2*points_fit[i][0];
        A(counter, 1 ) = 2*points_fit[i][1]; 
        A(counter, 2 ) = 2*points_fit[i][2]; 
        A(counter, 3 ) = 1;
        f[counter] = points_fit[i][0]* points_fit[i][0] ;
        f[counter]+=points_fit[i][1]* points_fit[i][1];
        f[counter]+=points_fit[i][2]* points_fit[i][2];
        counter++;

    }
    c = A.colPivHouseholderQr().solve(f);
}



void Calibrate::sphere_ransac(std::vector<cv::Vec3d>  points_sphere ){


double max_score = 0 ; 
std::vector<int> index_best_sphere ;  
for(int i =0 ;i<10000; i++)
{
    std::set<int> sphere_indecies;
    while(sphere_indecies.size()<4)
    {
        sphere_indecies.insert(rand() % points_sphere.size())  ; 
    }



    Eigen::VectorXd c;

    sphere_fit_set(points_sphere , c,sphere_indecies);
    double radius = std::sqrt(c[3] + c[0]*c[0] + c[1]*c[1] +c[2]*c[2] ); 

    

    double score =  0; 
    std::vector<int> agree_with_sphere ;  
    for(int j =0 ; j<points_sphere.size() ; j++)
    {
        cv::Vec3d curr_point = points_sphere[j] ;
        double distance = (curr_point[0] - c[0] )*(curr_point[0] - c[0] ) ;
        distance += (curr_point[1] - c[1] )*(curr_point[1] - c[1] ) ;
        distance += (curr_point[2] - c[2] )*(curr_point[2] - c[2] ) ;
        distance = std::sqrt(distance) ; 
        double sphere_distance = std::abs(radius-distance) ; 
        if(sphere_distance <0.001)
        {
            score +=1 ;
            agree_with_sphere.push_back(j);
            



        }





    }



    if (score>max_score)
    {
        max_score = score; 
        //index_best_sphere.clear() ; 
        //index_best_sphere.reserve(agree_with_sphere.size());
        //copy(agree_with_sphere.begin(), agree_with_sphere.end(), index_best_sphere.begin());
        index_best_sphere = agree_with_sphere; 

    }

    




}
    std::cout << "best score" << max_score << std::endl;
    Eigen::VectorXd c;
    sphere_fit_vector(points_sphere , c,index_best_sphere);
    double radius = std::sqrt(c[3] + c[0]*c[0] + c[1]*c[1] +c[2]*c[2] ); 
    std::cout << "radius" << radius  << std::endl;




}  



void Calibrate::find_interval(std::vector<std::tuple<double,double>> &intervals  ){
    bool before_true = true ; 
    int start_index = 0  ; 
    for(int i = 1;i<is_still.size() -1 ; i++ )
    {
        if(before_true)
        {
            if(!is_still[i])
            {
                double time_diff = time_of_images[i] - time_of_images[start_index] ; 
                if(time_diff > 1)
                {
                     intervals.push_back( std::make_tuple(time_of_images[start_index+1],time_of_images[i-1]) ); 
                }
                before_true = false;
            }
        }


        else{

            if(is_still[i])
            {
                before_true = true;
                start_index = i ; 
            }


        }

    }

    if(start_index = 0 && before_true )
    { 
        intervals.push_back(std::make_tuple(time_of_images[start_index+1],time_of_images[time_of_images.size()-1]) ) ; 
    }




}


void  Calibrate::calc_accel_bias(){

    std::ofstream MyFile("/code/accels.txt");
    std::vector<cv::Vec3d> no_movement_frames ;
    std::vector<std::tuple<double,double>> intervals ;
    find_interval(intervals) ; 
    for(int i = 0;i<intervals.size() ; i++ )
    {        

        double start = std::get<0>(intervals[i]) ; 
        double end = std::get<1>(intervals[i]) ; 
        auto upper1 = std::upper_bound(time_imu.begin(), time_imu.end(), start);
        auto upper2 = std::upper_bound(time_imu.begin(), time_imu.end(), end);
        int time_index_start = upper1 -time_imu.begin();
        int time_index_end = upper2 -time_imu.begin();

        for(int j = time_index_start ; j<time_index_end ; j++ )
        {
            cv::Vec3d accel = imu_accels[j];
            MyFile <<std::setprecision(64) <<  accel[0] << " " << accel[1] << " " << accel[2] << std::endl;
            no_movement_frames.push_back(accel);
        }







    }




    sphere_ransac(no_movement_frames );



    MyFile.close();




     





    







}