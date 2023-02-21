#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>
#include <fstream>



#define h_translation  0.01
#define h_rotation  0.01

void Calibrate::copyvec(cv::Vec3d &copyto , cv::Vec3d &empty)
{
    empty[0] = copyto[0] ; 
    empty[1] = copyto[1] ; 
    empty[2] = copyto[2] ; 



}



cv::Vec3d to_vec3d(cv::Mat const& m)
{
    return cv::Vec3d(m.at<double>(0), m.at<double>(1), m.at<double>(2));
}







void Calibrate::calc_imu_state( double start_time , double end_time ,cv::Vec3d & imu_translation , cv::Mat & imu_rotation)
{ 

            imu_rotation =  (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1); 
            //cv::Vec3d imu_translation ;
            imu_translation[0] = 0; 
            imu_translation[1] = 0;
            imu_translation[2] = 0;  

        
            cv::Vec3d imu_velocity  ;
            imu_velocity[0] = 0;
            imu_velocity[1] = 0;
            imu_velocity[2] = 0;
            double dt_imu = time_imu[2] -  time_imu[1] ; 
            std::cout << dt_imu << std::endl;
            cv::Vec3d gravity ;
            bool first = true; 
           for(int i = 0  ; i < time_imu.size()    ; i++)
            {


                if(time_imu[i] >= start_time && time_imu[i] <= end_time  )
                {
                if(first)
                {
                    gravity = imu_accels[i] - accel_bias ; 
                    first = false ; 

                }
                else{
                cv::Vec3d curr_angular_velocity = angular_velocities[i] -gyro_bias ; 
                 cv::Mat curr_rotation ; 
                cv::Rodrigues(curr_angular_velocity *dt_imu, curr_rotation );
                cv::Vec3d  curr_accel = imu_accels[i] -accel_bias ; 
                 
                imu_rotation = curr_rotation*imu_rotation;

                curr_accel = to_vec3d(imu_rotation*curr_accel) - gravity; 
                imu_translation = imu_translation + dt_imu*imu_velocity +  (0.5*dt_imu*dt_imu)*curr_accel  ; 
                //std::cout << curr_accel << std::endl;
                imu_velocity = imu_velocity +  curr_accel*dt_imu ; 
                }
                


                }


                if( time_imu[i] > end_time)
                {
                    break ; 
                }

            }
            







}



void cv_to_eig(cv::Mat &cv_mat ,Eigen::MatrixXd &mat )
{
    for (int i = 0 ; i<4;i++)
    {
        for(int j = 0 ; j <4 ; j++)
        {
             cv_mat.at<double>(i, j) = mat(i,j);
        }
    }
}



void create_transformation(cv::Vec3d &translation , cv::Vec3d &rotation_vec , cv::Mat &transformation)
{
    cv::Mat rotation ;
    cv::Rodrigues(rotation_vec , rotation) ;
    for (int i = 0 ; i<3;i++)
    {
        for(int j = 0 ; j <3 ; j++)
        {
             transformation.at<double>(i, j) = rotation.at<double>(i, j);

        }
    }
    transformation.at<double>(0,3)=  translation[0] ; 
    transformation.at<double>(1,3) =  translation[1] ; 
    transformation.at<double>(2,3) =  translation[2] ; 
    transformation.at<double>(3,3) = 1 ; 


}



void extract_rot_and_trans (cv::Vec3d &translation , cv::Vec3d  &rotation_vec , cv::Mat &transformation)
{
    cv::Mat rotation  = cv::Mat::zeros(4 ,4,CV_64F);;
    for (int i = 0 ; i<3;i++)
    {
        for(int j = 0 ; j <3 ; j++)
        {
            rotation.at<double>(i, j) =  transformation.at<double>(i, j) ;
        }
    }

    cv::Rodrigues(rotation,rotation_vec) ; 
    translation[0] = transformation.at<double>(0,3) ; 
     translation[1] = transformation.at<double>(1,3) ; 
    translation[2] = transformation.at<double>(2,3) ; 




}








void log_se3(cv::Vec3d &translation , cv::Vec3d &rotation , cv::Mat &transformation)
{
    cv::Mat rotation_matrix =  cv::Mat::zeros(3 ,3,CV_64F);; 


    for(int i = 0 ; i<3; i++)
    {
        for(int j = 0;j<3;j++)
        {
           rotation_matrix.at<double>(i,j) =  transformation.at<double>(i,j) ; 
        }
    }
    cv::Rodrigues(rotation_matrix,rotation ) ;
    cv::Mat skew_rot = cv::Mat::zeros(3 ,3,CV_64F);
    skew_rot.at<double>(1,0) = rotation[2] ; 
    skew_rot.at<double>(2,0) =  -1*rotation[1] ;
    skew_rot.at<double>(0,1) = -1*rotation[2] ;  
     skew_rot.at<double>(2,1) = rotation[0] ;  
     skew_rot.at<double>(0,2) = rotation[1] ; 
     skew_rot.at<double>(1,2) = -1*rotation[0] ;  
    double rot_trace = cv::trace(rotation_matrix)[0] ; 
    double thetha = std::acos((rot_trace -1 )/2) ;

    cv::Mat v = cv::Mat::eye(3 , 3  , CV_64F); 
    if(thetha!=0)
    {
    v+=  ( (1-std::cos(thetha))/(thetha*thetha)) *skew_rot ;
    v+=  (  (thetha - std::sin(thetha) )/ (thetha*thetha*thetha ) )*skew_rot*skew_rot  ;  
    }

    cv::Vec3d p ; 
    p[0] = transformation.at<double>(0,3) ; 
    p[1] = transformation.at<double>(1,3) ; 
    p[2] = transformation.at<double>(2,3) ; 
    translation = to_vec3d(v.inv() * p ) ; 








}
void exp_se3(cv::Vec3d &translation , cv::Vec3d &rotation , cv::Mat &transformation)
{
    cv::Mat rotation_matrix ;
    cv::Rodrigues(rotation , rotation_matrix) ; 
    double rot_trace = cv::trace(rotation_matrix)[0] ; 

    double thetha = std::acos((rot_trace -1 )/2) ;
    cv::Mat skew_rot = cv::Mat::zeros(3 ,3,CV_64F);
    skew_rot.at<double>(1,0) = rotation[2] ; 
    skew_rot.at<double>(2,0) =  -1*rotation[1] ;
    skew_rot.at<double>(0,1) = -1*rotation[2] ;  
     skew_rot.at<double>(2,1) = rotation[0] ;  
     skew_rot.at<double>(0,2) = rotation[1] ; 
     skew_rot.at<double>(1,2) = -1*rotation[0] ;   


    
    cv::Mat v = cv::Mat::eye(3, 3 , CV_64F); 
    if(thetha!=0)
    {
    v +=  ( (1-std::cos(thetha))/(thetha*thetha)) *skew_rot ;
    v+=  (  (thetha - std::sin(thetha))/(thetha*thetha*thetha ))    *skew_rot*skew_rot  ; 
    }

    cv::Vec3d p =   to_vec3d(v*translation) ; 
    transformation = cv::Mat::zeros(4 ,4,CV_64F);
    for(int i = 0 ; i<3; i++)
    {
        for(int j = 0;j<3;j++)
        {
            transformation.at<double>(i,j) = rotation_matrix.at<double>(i,j) ; 
        }
    }


    for(int i = 0 ; i<3; i++)
    {
        transformation.at<double>(i,3) =  p[i] ;   
    }
    transformation.at<double>(3,3) = 1 ;










}



void calc_res(cv::Vec3d &i2c_trans , cv::Vec3d &i2c_rot , std::tuple<cv::Vec3d , cv::Vec3d> &first_pose , 
std::tuple<cv::Vec3d , cv::Vec3d> &last_pose  , cv::Vec3d  &imu_translation ,cv::Vec3d  &imu_rotation,cv::Mat &residual , int i )
{
        cv::Mat i2c  ; 
        exp_se3(i2c_trans,i2c_rot,i2c ) ; 

        cv::Mat c1  ; 
        exp_se3(std::get<1>(first_pose ) ,std::get<0>(first_pose ) ,c1 ) ; 



        cv::Mat c2 ; 
        exp_se3(std::get<1>(last_pose ) ,std::get<0>(last_pose ) ,c2  ) ; 




        cv::Mat i_measure  ; 
        exp_se3(imu_translation ,imu_rotation ,i_measure ) ;

        cv::Mat c2c1 = c1.inv()* c2 ;
        cv::Mat mat_mul = c2c1 * i2c*  i_measure * i2c.inv() ; 
        cv::Vec3d rot_error ; 
        cv::Vec3d trans_error  ;
        log_se3(trans_error , rot_error , mat_mul ) ;




        residual.at<double>(6*i+ 0) = rot_error[0] ; 
        residual.at<double>(6*i + 1) =  rot_error[1] ;  
        residual.at<double>(6*i +2) = rot_error[2] ;   
        residual.at<double>(6*i +3) = trans_error[0] ; 
        residual.at<double>(6*i +4) = trans_error[1] ; 
        residual.at<double>(6*i +5) = trans_error[2] ; 
}






void Calibrate::calc_extrinsic()
{   

    std::vector<std::tuple<double,double>> intervals;
    std::vector<int> end_index ; 
    Calibrate::find_interval(intervals   , end_index);


    std::cout << "intervals" << intervals.size() << std::endl;  
    cv::Mat residual = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
    cv::Mat jacobian= cv::Mat::zeros(6*intervals.size() ,6 ,CV_64F);
    cv::Vec3d i2c_rot = cv::Vec3d(0,0,0) ; 
    cv::Vec3d i2c_trans = cv::Vec3d(0,0,0) ; 
    bool first = true ; 
    for(int i = 0; i<intervals.size() ; i++)
    {



        double camera_tvec_movment = 0;
        double camera_rvec_movment = 0 ; 

        double c1_time = std::get<1>(intervals[i]) ; 
        int camera_index = end_index[i] +1 ; 
        std::tuple<cv::Vec3d ,cv::Vec3d > first_pose =   trajectory_camera[end_index[i]] ;  


        while(camera_tvec_movment < 0.3 && camera_rvec_movment < 0.3)
        {
            std::tuple<cv::Vec3d ,cv::Vec3d > cm_pose =   trajectory_camera[camera_index] ; 
            camera_tvec_movment = cv::norm(std::get<1>(cm_pose )     -   std::get<1>(first_pose) )  ; 

            cv::Mat r1 ;
            cv::Rodrigues(std::get<0>(first_pose) , r1 ) ; 
            cv::Mat r2 ;
            cv::Rodrigues(std::get<0>(cm_pose) , r2 ) ; 
            cv::Vec3d rvec_movment ; 
            cv::Rodrigues(r1.t() * r2  , rvec_movment) ;
            camera_rvec_movment = cv::norm(rvec_movment)  ; 
            camera_index++; 


        }



       //std::tuple<double,double >  time_measure = std::make_tuple(time_of_images[end_index[i]] , time_of_images[camera_index] ); 
        cv::Mat imu_rotation_mat;
        cv::Vec3d imu_translation;
        calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat  ) ; 
        std::cout << "index "<< end_index[i]<<"," <<  camera_index  <<"  trans imu " << cv::norm(imu_translation) << std::endl ; 
        std::tuple<cv::Vec3d ,cv::Vec3d > last_pose =   trajectory_camera[camera_index] ;
        cv::Mat residual = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
        cv::Vec3d imu_rotation ; 
        cv::Rodrigues(imu_rotation_mat , imu_rotation);
        calc_res(i2c_trans , i2c_rot , first_pose , last_pose  , imu_translation ,imu_rotation, residual , i ) ; 
        std::cout << residual << std::endl;
        for(int j = 0 ;  j<3 ; j++)
        {
            

            i2c_trans[j] += h_translation ;
            cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
            calc_res(i2c_trans , i2c_rot , first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ;
            i2c_trans[j] -= h_translation ;
            for(int k = 0 ;k<6 ; k++)
            {




                int res_index = 6* i + k;
                double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index))/h_translation ; 
                //std::cout << derrivitive << std::endl; 
                jacobian.at<double>(res_index,j) = derrivitive ;
            }



        }

        for(int j = 0 ;  j<3 ; j++)
        {



            i2c_rot[j] += h_rotation ;
            cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
            calc_res(i2c_trans , i2c_rot , first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ;
            i2c_rot[j] -= h_rotation ;
            for(int k = 0 ;k<6 ; k++)
            {





                int res_index = 6* i + k; 

                double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index)) /h_rotation ; 

                jacobian.at<double>(res_index,j+3) = derrivitive ; 

            }


        }





        


    }




    residual = -1*residual ; 
    cv::Mat delta ; 
    cv::solve(jacobian.t() * jacobian , jacobian.t() *residual , delta);
    std::cout << delta << std::endl;



}





void start_calibration(Calibrate * calibrate_instance)
{


    //int flip =  std::round(4000/7) 
    while(calibrate_instance->image_queue.size() < 1200 )
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
