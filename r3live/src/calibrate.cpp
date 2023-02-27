#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <calibrate.hpp>
#include <fstream>



#define h_translation  1e-5
#define h_rotation  1e-5
#define h_gyro_bias  1e-5
#define h_accel_bias  1e-5
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







void Calibrate::calc_imu_state( double start_time , double end_time ,cv::Vec3d & imu_translation , cv::Mat & imu_rotation ,cv::Mat &camera2charuco_rotation  )
{ 
            cv::Mat i2c_rotation_mat ;

            cv::Rodrigues(i2c_rot, i2c_rotation_mat);

            cv::Mat char2w_rotation_matrix ;
            cv::Rodrigues(cha2w_rot,  char2w_rotation_matrix);
            cv::Mat i12w = char2w_rotation_matrix*camera2charuco_rotation *i2c_rotation_mat ; 
            //cv::Vec3d imu_translation ;
            imu_translation[0] = 0; 
            imu_translation[1] = 0;
            imu_translation[2] = 0;  
            imu_rotation = (cv::Mat_<double>(3,3) <<1  , 0 , 0,  0 , 1, 0 ,0 ,0,1);
            cv::Vec3d imu_velocity  ;
            imu_velocity[0] = 0;
            imu_velocity[1] = 0;
            imu_velocity[2] = 0;
            double dt_imu = time_imu[2] -  time_imu[1] ; 
            cv::Vec3d gravity = cv::Vec3d(0,0,9.823);
            bool first = true ; 

           for(int i = 0  ; i < time_imu.size()    ; i++)
            {


                if(time_imu[i] >= start_time && time_imu[i] <= end_time  )
                {



                if(!first)
                {
                cv::Vec3d curr_angular_velocity = angular_velocities[i] -gyro_bias ; 
                cv::Mat curr_rotation  ; 
                cv::Rodrigues(curr_angular_velocity *dt_imu, curr_rotation );
                cv::Vec3d  curr_accel = imu_accels[i] -accel_bias ; 
                 
                imu_rotation = curr_rotation * imu_rotation ;

                curr_accel = to_vec3d(i12w*imu_rotation*curr_accel) - gravity; 
                imu_translation = imu_translation + dt_imu*imu_velocity +  (0.5*dt_imu*dt_imu)*curr_accel  ; 
                //std::cout << curr_accel << std::endl;
                imu_velocity = imu_velocity +  curr_accel*dt_imu ; 
                }
                else
                {
                    first =false;
                }
                
                }


                if( time_imu[i] > end_time)
                {
                    break ; 
                }

            }
            //std::cout << "trans " << imu_translation<< std::endl; 
            //imu_translation =  to_vec3d(i12w.t() * imu_translation) ; 
            //std::cout << "trans " << imu_translation<< std::endl; 
            







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
    //double rot_trace = cv::trace(rotation_matrix)[0] ; 
    double thetha = cv::norm(rotation) ; //std::acos((rot_trace -1 )/2) ;

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
    //double rot_trace = cv::trace(rotation_matrix)[0] ; 

    //double thetha = std::acos((rot_trace -1 )/2) ;


    double thetha = cv::norm(rotation) ;  //std::acos((rot_trace -1 )/2) ;



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



void Calibrate::calc_res( std::tuple<cv::Vec3d , cv::Vec3d> &first_pose , 
std::tuple<cv::Vec3d , cv::Vec3d> &last_pose  , cv::Vec3d  &imu_translation ,cv::Vec3d  &imu_rotation,cv::Mat &residual , int i )
{

        cv::Mat  ch2world ; 
        cv::Rodrigues(cha2w_rot , ch2world);
        cv::Mat i2c  ; 
        //exp_se3(i2c_trans,i2c_rot,i2c ) ;
         cv::Rodrigues(i2c_rot ,i2c ) ; 

        cv::Mat c1_rot ; 
        //exp_se3(std::get<1>(last_pose ) ,std::get<0>(last_pose ) ,c2  ) ; 
        cv::Rodrigues(std::get<0>(first_pose ) ,c1_rot ) ; 
        cv::Vec3d c1_translation ;
        c1_translation = std::get<1>(first_pose ) ; 
        c1_rot= ch2world*c1_rot;



        cv::Mat c2_rot ; 
        //exp_se3(std::get<1>(last_pose ) ,std::get<0>(last_pose ) ,c2  ) ; 
        cv::Rodrigues(std::get<0>(last_pose ) ,c2_rot ) ; 
        cv::Vec3d c2_translation ;
        c2_rot= ch2world*c2_rot;
        c2_translation = std::get<1>(last_pose ) ; 



        cv::Mat c2c1 = c1_rot.t()* c2_rot ;
        
        
         

        //cv::Mat i_measure  ; 
        cv::Mat imu_rot  ; 
        //exp_se3(imu_translation ,imu_rotation ,i_measure ) ;
        cv::Rodrigues(imu_rotation , imu_rot) ;



        cv::Mat mat_mul = ch2world*c2c1 * i2c*imu_rot ;

        //cv::Mat c2c1 = c1.inv()* c2 ;
        //cv::Mat mat_mul = c2c1 * i2c*  i_measure * i2c.inv() ; 
        cv::Vec3d rot_error ; 
        cv::Vec3d trans_error  ;
        //log_se3(trans_error , rot_error , mat_mul ) ;




        cv::Rodrigues(mat_mul , rot_error) ; 


        //std::cout << "2 " << std::endl;
        
        trans_error =  to_vec3d(ch2world*(c2_translation))     - to_vec3d(ch2world*c1_translation) - to_vec3d(i2c*imu_translation )- i2c_trans ; 





        



        residual.at<double>(6*i+ 0) = rot_error[0] ; 
        residual.at<double>(6*i + 1) = rot_error[1] ;  
        residual.at<double>(6*i +2) = rot_error[2] ;   
        residual.at<double>(6*i +3)  = trans_error[0] ; 
        residual.at<double>(6*i +4) = trans_error[1]  ; 
        residual.at<double>(6*i +5) = trans_error[2] ; 


}






void Calibrate::calc_extrinsic()
{   

    std::vector<std::tuple<double,double>> intervals;
    std::vector<int> end_index ; 
    Calibrate::find_interval(intervals   , end_index);


    std::cout << "intervals" << intervals.size() << std::endl;  






    for(int p = 0 ; p<100 ; p++)
    {





    cv::Mat residual = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
    cv::Mat jacobian= cv::Mat::zeros(6*intervals.size() ,15 ,CV_64F);
    std::tuple<cv::Vec3d ,cv::Vec3d > last_pose ;

    bool first = true ; 





    cv::Mat camera2charu_rotation ;

    cv::Vec3d imu_translation_copy ;
    std::tuple<cv::Vec3d ,cv::Vec3d > first_pose  ; 



    for(int i = 0; i<intervals.size() ; i++)
    {



        double camera_tvec_movment = 0;
        double camera_rvec_movment = 0 ; 

        double c1_time = std::get<1>(intervals[i]) ; 
        int camera_index = end_index[i] +1 ; 
        first_pose =   trajectory_camera[end_index[i]] ;  


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

        cv::Rodrigues(std::get<0>(first_pose) ,  camera2charu_rotation );

       //std::tuple<double,double >  time_measure = std::make_tuple(time_of_images[end_index[i]] , time_of_images[camera_index] ); 
        cv::Mat imu_rotation_mat;
        cv::Vec3d imu_translation;                                   
        calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat , camera2charu_rotation  ) ; 
        //std::cout << "index "<< end_index[i]<<"," <<  camera_index  <<"  trans imu " << cv::norm(imu_translation) << std::endl ; 
         last_pose =   trajectory_camera[camera_index] ;
        cv::Vec3d imu_rotation ; 
        cv::Rodrigues(imu_rotation_mat , imu_rotation);


        imu_translation_copy[0] = imu_translation[0] ; 
        imu_translation_copy[1] = imu_translation[1] ;
        imu_translation_copy[2] = imu_translation[2] ;





        calc_res(first_pose , last_pose  , imu_translation ,imu_rotation, residual , i ) ; 

        //std::cout << residual << std::endl;





















        // derrivative translation
        for(int j = 0 ;  j<3 ; j++)
        {
            

            i2c_trans[j] += h_translation ;
            cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
            calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat , camera2charu_rotation  ) ; 
             cv::Rodrigues(imu_rotation_mat , imu_rotation);
            calc_res( first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ;
            i2c_trans[j] -= h_translation ;

            for(int k = 0 ;k<6 ; k++)
            {


                int res_index = 6* i + k;
                double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index))/h_translation ; 
                //std::cout << derrivitive << std::endl; 
                jacobian.at<double>(res_index,j+3) = derrivitive ;
            }



        }

        // derrivative rotation
        for(int j = 0 ;  j<3 ; j++)
        {



            i2c_rot[j] += h_rotation ;
            cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
            calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat , camera2charu_rotation ) ; 
             cv::Rodrigues(imu_rotation_mat , imu_rotation);
            calc_res(first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ;
            i2c_rot[j] -= h_rotation ;
            for(int k = 0 ;k<6 ; k++)
            {





                int res_index = 6* i + k; 

                double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index)) /h_rotation ; 

                jacobian.at<double>(res_index,j) = derrivitive ; 

            }


        }


        //derrivative gyro bias 
    for(int k = 0 ; k<3;k++)
    {

        gyro_bias[k]+=h_gyro_bias ; 
        calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat , camera2charu_rotation  ) ;
         cv::Rodrigues(imu_rotation_mat , imu_rotation);
        cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
        calc_res( first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ; 
        gyro_bias[k]-=h_gyro_bias ; 

        for(int j = 0 ; j<6;j++)
        {
            int res_index = 6* i + j;
            double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index))/h_gyro_bias ; 
            jacobian.at<double>(res_index, 6+k ) = derrivitive ;

        }

    }


    //derivative accel bias
    for(int k =0 ; k<3;k++)
        {
        accel_bias[k]+=h_accel_bias ; 
        calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat , camera2charu_rotation  ) ; 
         cv::Rodrigues(imu_rotation_mat , imu_rotation);
        cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F);
        calc_res( first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ; 
        accel_bias[k]-=h_accel_bias ; 

         for(int j = 0 ; j<6;j++)
        {
            int res_index = 6* i + j;
            double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index))/h_accel_bias ; 
            jacobian.at<double>(res_index, 9+k ) = derrivitive ;

        }
        }














   //derivative charuco to world
    for(int k =0 ; k<3;k++)
        {
        cha2w_rot[k]+=h_rotation ; 
        calc_imu_state( time_of_images[end_index[i]]  , time_of_images[camera_index],  imu_translation ,  imu_rotation_mat ,  camera2charu_rotation  ) ;
         cv::Rodrigues(imu_rotation_mat , imu_rotation);
        cv::Mat temp_res = cv::Mat::zeros(6*intervals.size() ,1,CV_64F); 
        calc_res( first_pose , last_pose  , imu_translation ,imu_rotation, temp_res , i ) ; 
        cha2w_rot[k]-=h_rotation ; 
         for(int j = 0 ; j<6;j++)
        {
            int res_index = 6* i + j;
            double derrivitive= (temp_res.at<double>(res_index) - residual.at<double>(res_index))/h_accel_bias ; 
            jacobian.at<double>(res_index, 12+k ) = derrivitive ;

        }

        }
















        









        


    }




    residual = -1*residual ; 
    cv::Mat delta ;
    double min_jacob  ; 
    double max_jacob ; 

    cv::minMaxIdx(jacobian, &min_jacob , &max_jacob);
    cv::solve(jacobian.t() * jacobian , jacobian.t() *residual , delta);
    std::cout << "residual" << cv::norm(residual) << std::endl;
    delta = 0.1*delta ; 
    std::cout << "delta" << cv::norm(delta) << std::endl;
    for(int i = 0 ; i <3 ; i++ )
    {
        i2c_rot[i] +=delta.at<double>(i);
    }
    for(int i = 0 ; i <3 ; i++ )
    {
        i2c_trans[i] +=delta.at<double>(3+i);
    }
    for(int i = 0 ; i <3 ; i++ )
    {
        gyro_bias[i] +=delta.at<double>(6+i);
    }
    for(int i = 0 ; i <3 ; i++ )
    {
        accel_bias[i] +=delta.at<double>(9+i);
    }
    for(int i = 0 ; i <3 ; i++ )
    {
        cha2w_rot[i] +=delta.at<double>(12+i);
    }


   // cv::Mat cha2w_rot_mat ;
    //cv::Rodrigues(cha2w_rot,cha2w_rot_mat) ;
    //cv::Mat c2_to_cha ; 
    //cv::Rodrigues(std::get<0>(last_pose ) , c2_to_cha ) ;
    //cv::Mat camera_trans = cha2w_rot_mat*camera2charu_rotation*  std::get<1>(last_pose)  -     cha2w_rot_mat*c2_to_cha*std::get<1>(first_pose )   ;
    //cv::Mat i2c ;
    //exp_se3(i2c_trans,i2c_rot,i2c) ; 
    //cv::Mat imu_trans = (cv::Mat_<double>(4,1) <<imu_translation_copy[0]  , imu_translation_copy[1] ,  imu_translation_copy[2] , 1);
   // imu_trans = i2c*imu_trans ; 
    //imu_translation_copy = to_vec3d(imu_trans) ; 
    //imu_translation_copy[0]+= i2c.at<double>(0,3) ; 
    //imu_translation_copy[1]+= i2c.at<double>(1,3) ; 
    //imu_translation_copy[2]+= i2c.at<double>(2,3) ; 
    //cv::Mat integration_in_camera = i2c*imu_translation_copy ; 

    //std::cout << "xy distance " << cv::norm(imu_translation_copy -  to_vec3d(camera_trans)) << std::endl;
    

    







    }


    cv::Mat cha2w_rot_matrix ; 
    cv::Rodrigues(cha2w_rot , cha2w_rot_matrix) ;
    std::cout << cha2w_rot_matrix << std::endl;


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
