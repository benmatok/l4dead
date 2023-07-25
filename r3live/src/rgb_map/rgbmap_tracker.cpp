/*
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored,
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored,
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package."
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include "rgbmap_tracker.hpp"
#include <unsupported/Eigen/Splines>
#include "profc.h"
Rgbmap_tracker::Rgbmap_tracker()
{
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.05);
    if (m_lk_optical_flow_kernel == nullptr)
    {
        m_lk_optical_flow_kernel = std::make_shared<LK_optical_flow_kernel>(cv::Size(21, 21), 3, criteria,
                                                                            cv_OPTFLOW_LK_GET_MIN_EIGENVALS);
    }
}

void Rgbmap_tracker::update_and_append_track_pts(std::shared_ptr<Image_frame> &img_pose, Global_map &map_rgb,
                                                 double mini_dis, int minimum_frame_diff)
{
    Common_tools::Timer tim;
    tim.tic();
    double u_d, v_d;
    int u_i, v_i;
    double max_allow_repro_err =   2.0 * img_pose->m_img_cols / 320.0;
    Hash_map_2d<int, float> map_2d_pts_occupied;

    for (auto it = m_map_rgb_pts_in_last_frame_pos.begin(); it != m_map_rgb_pts_in_last_frame_pos.end();)
    {
        RGB_pts *rgb_pt = ((RGB_pts *)it->first);
        vec_3 pt_3d = ((RGB_pts *)it->first)->get_pos();
        int res = img_pose->project_3d_point_in_this_img(pt_3d, u_d, v_d, nullptr, 1.0);
        u_i = std::round(u_d / mini_dis) * mini_dis;
        v_i = std::round(v_d / mini_dis) * mini_dis;

        double error = vec_2(u_d - it->second.x, v_d - it->second.y).norm();

        if (error > max_allow_repro_err)
        {
            // cout << "Remove: " << vec_2(it->second.x, it->second.y).transpose() << " | " << vec_2(u, v).transpose()
            // << endl;
            rgb_pt->m_is_out_lier_count++;
            if (rgb_pt->m_is_out_lier_count > 1 || (error > max_allow_repro_err * 2))
            // if (rgb_pt->m_is_out_lier_count > 3)
            {
                rgb_pt->m_is_out_lier_count = 0; // Reset
                it = m_map_rgb_pts_in_last_frame_pos.erase(it);
                continue;
            }
        }
        else
        {
            rgb_pt->m_is_out_lier_count = 0;
        }

        if (res)
        {
            double depth = (pt_3d - img_pose->m_pose_w2c_t).norm();
            if (map_2d_pts_occupied.if_exist(u_i, v_i) == false)
            {
                map_2d_pts_occupied.insert(u_i, v_i, depth);
                // it->second = cv::Point2f(u, v);
            }
        }
        else
        {
            // m_map_rgb_pts_in_last_frame_pos.erase(it);
        }
        it++;
    }

    int new_added_pts = 0;

    tim.tic("Add");
    while (map_rgb.m_updated_frame_index < img_pose->m_frame_idx - minimum_frame_diff)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
    }
    map_rgb.m_mutex_pts_vec->lock();
    int new_add_pt = 0;

    if (map_rgb.m_pts_rgb_vec_for_projection != nullptr)
    {
        int pt_size = map_rgb.m_pts_rgb_vec_for_projection->size();
        for (int i = 0; i < pt_size; i++)
        {
            if (m_map_rgb_pts_in_last_frame_pos.find((*map_rgb.m_pts_rgb_vec_for_projection)[i].get()) !=
                m_map_rgb_pts_in_last_frame_pos.end())
            {
                continue;
            }
            vec_3 pt_3d = (*map_rgb.m_pts_rgb_vec_for_projection)[i]->get_pos();
            int res = img_pose->project_3d_point_in_this_img(pt_3d, u_d, v_d, nullptr, 1.0);
            u_i = std::round(u_d / mini_dis) * mini_dis;
            v_i = std::round(v_d / mini_dis) * mini_dis;
            // vec_3 rgb_color = img_pose->get_rgb(u, v);
            // double grey = img_pose->get_grey_color(u, v);
            // (*map_rgb.m_pts_rgb_vec_for_projection)[i]->update_gray(grey);
            // (*map_rgb.m_pts_rgb_vec_for_projection)[i]->update_rgb(rgb_color);
            if (res)
            {
                double depth = (pt_3d - img_pose->m_pose_w2c_t).norm();
                if (map_2d_pts_occupied.if_exist(u_i, v_i) == false)
                {
                    map_2d_pts_occupied.insert(u_i, v_i, depth);
                    m_map_rgb_pts_in_last_frame_pos[(*map_rgb.m_pts_rgb_vec_for_projection)[i].get()] =
                        cv::Point2f(u_d, v_d);
                    new_added_pts++;
                }
            }
       if (m_map_rgb_pts_in_last_frame_pos.size() >= m_maximum_vio_tracked_pts)
        {
            break;
        }
            new_add_pt++;
        }
        // cout << "Tracker new added pts = " << new_added_pts << " |  " << map_rgb.m_pts_rgb_vec_for_projection->size()
        //      << " | " << img_pose->m_frame_idx << " | " << map_rgb.m_updated_frame_index
        //      << ", cost = " << tim.toc("Add") << endl;
    }

    map_rgb.m_mutex_pts_vec->unlock();
    update_last_tracking_vector_and_ids();
    // cout << "Update points cost time = " << tim.toc() << endl;
}

void Rgbmap_tracker::reject_error_tracking_pts(std::shared_ptr<Image_frame> &img_pose, double dis)
{
    double u, v;
    int remove_count = 0;
    int total_count = m_map_rgb_pts_in_current_frame_pos.size();
    // cout << "Cam mat: " <<img_pose->m_cam_K << endl;
    // cout << "Image pose: ";
    // img_pose->display_pose();
    scope_color(ANSI_COLOR_BLUE_BOLD);
    for (auto it = m_map_rgb_pts_in_current_frame_pos.begin(); it != m_map_rgb_pts_in_current_frame_pos.end(); it++)
    {
        cv::Point2f predicted_pt = it->second;
        vec_3 pt_3d = ((RGB_pts *)it->first)->get_pos();
        int res = img_pose->project_3d_point_in_this_img(pt_3d, u, v, nullptr, 1.0);
        if (res)
        {
            if ((fabs(u - predicted_pt.x) > dis) || (fabs(v - predicted_pt.y) > dis))
            {
                // Remove tracking pts
                m_map_rgb_pts_in_current_frame_pos.erase(it);
                remove_count++;
            }
        }
        else
        {
            // cout << pt_3d.transpose() << " | ";
            // cout << "Predicted: " << vec_2(predicted_pt.x, predicted_pt.y).transpose() << ", measure: " << vec_2(u,
            // v).transpose() << endl;
            m_map_rgb_pts_in_current_frame_pos.erase(it);
            remove_count++;
        }
    }
    cout << "Total pts = " << total_count << ", rejected pts = " << remove_count << endl;
}

// inline void image_equalize(cv::Mat &img, int amp)
// {
//     cv::Mat img_temp;
//     cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
//     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
//     // Equalize gray image.
//     clahe->apply(img, img_temp);
//     img = img_temp;
// }

// inline cv::Mat equalize_color_image_ycrcb(cv::Mat &image)
// {
//     cv::Mat hist_equalized_image;
//     cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);

//     //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
//     std::vector<cv::Mat> vec_channels;
//     cv::split(hist_equalized_image, vec_channels);

//     //Equalize the histogram of only the Y channel
//     // cv::equalizeHist(vec_channels[0], vec_channels[0]);
//     image_equalize( vec_channels[0], 2 );
//     cv::merge(vec_channels, hist_equalized_image);
//     cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
//     return hist_equalized_image;
// }


struct Point
{
    double x;
    double y;
    double z;
};


float get_interpulation_value(cv::Mat &image , int row , int col ,int kernel_size , float x , float y   ) 
{
    



    if(x >= image.rows  -2 ||  y <= 0 || x <= 0  || y >=image.cols  -2 )
    {
        return image.at<float>(row,col) ; 

    }




    int rows = image.rows ; 
    int cols = image.cols ; 

    int p1_x =  floor(x ) ; 
    int p1_y =  floor(y ) ; 
    float p1_z  =  image.at<float>(p1_x,p1_y) ; 



    int p2_x =  ceil(x+1 ) ; 
    int p2_y =   floor(y ) ; 
    float p2_z  =  image.at<float>(p2_x,p2_y) ;


    int p3_y =  ceil(y+1 ) ; 
    int p3_x =   floor(x ) ; 
    float p3_z  =  image.at<float>(p3_x,p3_y) ;

    int p4_y =  ceil(y+1 ) ; 
    int p4_x =   ceil(x+1 ) ; 

     float p4_z  =  image.at<float>(p4_x,p4_y) ;



    // Check if the sample points lie on a horizontal line
    
    float z1 = p1_z + (x - p1_x) / (p2_x - p1_x) * (p2_z - p1_z);
    float z2 = p3_z + (x - p3_x) / (p4_x - p3_x) * (p4_z - p3_z);
    


    float value = z1 + (y - p1_y) / (p3_y - p1_y) * (z2 - z1);

    return value;



    

 



}





void Rgbmap_tracker::register_images(cv::Mat &new_gray , cv::Mat &old_gray  , cv::Mat &deform_row , cv::Mat &deform_col  , int num_iter , int kernel_size)
{
   // PROFC_NODE("register_images");

    cv::Mat x_kernel =  (cv::Mat_<float>(1, 3) << -0.5f, 0.0f, 0.5f);
    cv::Mat  grad_x_old ;  
    cv::filter2D(old_gray, grad_x_old, -1, x_kernel); 
    cv::Mat grad_y_old  ;
    cv::filter2D(old_gray, grad_y_old, -1, x_kernel.t() );


    for (int iter = 0; iter < num_iter; iter++)
    {   
        for (int i = 0; i < new_gray.rows; i++)
        {
            for (int j = 0; j < new_gray.cols; j++)
            {
                 
                

                

                float  new_gray_value  = get_interpulation_value(new_gray , i ,  j ,   7 , i +  deform_row.at<float>(i,j) ,j +  deform_col.at<float>(i,j)   )  ;
                float grad_x = grad_x_old.at<float>(i, j);
                float grad_y = grad_y_old.at<float>(i, j);

        
                float i_minus_jt = old_gray.at<float>(i, j) - new_gray_value;
                float grad_norm_2 = grad_x * grad_x + grad_y * grad_y;
                float alpha = 1 ;

                float temp_factor = i_minus_jt/(grad_norm_2 + i_minus_jt * i_minus_jt + 1e-4);
                float v_row =  grad_y * temp_factor;
                float v_col =  grad_x * temp_factor;

                deform_row.at<float>(i,j) += v_row ;
                deform_col.at<float>(i,j) += v_col ;
                

            }
        }

        cv::Size blurSize(kernel_size, kernel_size );  // Kernel size for blurring (adjust as needed)
        cv::Mat blurred_deform_row ; 
        cv::Mat blurred_deform_col ; 
        cv::boxFilter(deform_row, blurred_deform_row, -1, blurSize);
        cv::boxFilter(deform_col, blurred_deform_col, -1, blurSize);
        blurred_deform_row.copyTo(deform_row);
        blurred_deform_col.copyTo(deform_col);


        
    }


}






void Rgbmap_tracker::demon_scale(cv::Mat &new_gray , cv::Mat &old_gray , cv::Mat &deform_row , cv::Mat &deform_col ,  int scale  , int iter , int kernel_size , cv::Mat &deform_row_before , cv::Mat &deform_col_before )  
{
    //PROFC_NODE("demon_scale");
    cv::Size new_size = cv::Size(new_gray.cols / scale, new_gray.rows / scale ) ; 
    cv::Size old_size = cv::Size(new_gray.cols , new_gray.rows  ) ; 
    cv::Mat new_gray_scale ;
    cv::Mat old_gray_scale ;
    deform_row_before = deform_row_before  / scale ; 
    deform_col_before = deform_col_before  / scale ; 


    cv::resize(new_gray, new_gray_scale,new_size  , cv::INTER_LINEAR );
    cv::resize(old_gray, old_gray_scale,new_size  , cv::INTER_LINEAR );

     //cv::Mat deform_row_scale ; // = cv::Mat::zeros(old_gray_scale.rows, old_gray_scale.cols, CV_32F)  ;
    //cv::Mat deform_col_scale ;//=  cv::Mat::zeros(old_gray_scale.rows, old_gray_scale.cols, CV_32F)  ;  ;
     cv::resize(deform_row_before, deform_row,new_size  , cv::INTER_LINEAR );
     cv::resize(deform_col_before, deform_col,new_size  , cv::INTER_LINEAR );
     register_images(new_gray_scale , old_gray_scale  , deform_row , deform_col  , iter , kernel_size ) ;
    deform_row = deform_row*scale ; 
    deform_col = deform_col*scale ; 




}
















Eigen::Vector3i jetColorMap2(double value)
{
    int red = 0;
    int green = 0;
    int blue = 0;

    if (value < 0.0)
        value = 0.0;
    else if (value > 1.0)
        value = 1.0;

    if (value <= 0.125)
    {
        red = 0;
        green = 0;
        blue = static_cast<int>(0.5 + 0.5 * (value * 8.0) * 255.0);
    }
    else if (value <= 0.375)
    {
        red = 0;
        green = static_cast<int>((value - 0.125) * 4.0 * 255.0);
        blue = 255;
    }
    else if (value <= 0.625)
    {
        red = static_cast<int>((value - 0.375) * 4.0 * 255.0);
        green = 255;
        blue = static_cast<int>(255.0 - (value - 0.375) * 4.0 * 255.0);
    }
    else if (value <= 0.875)
    {
        red = 255;
        green = static_cast<int>(255.0 - (value - 0.625) * 4.0 * 255.0);
        blue = 0;
    }
    else
    {
        red = static_cast<int>(255.0 - (value - 0.875) * 4.0 * 255.0);
        green = 0;
        blue = 0;
    }

    return Eigen::Vector3i(red, green, blue);
}








void Rgbmap_tracker::demon_track_image(cv::Mat &curr_img, const std::vector<cv::Point2f> &last_tracked_pts, std::vector<cv::Point2f> &curr_tracked_pts, std::vector<uchar> &status ,std::shared_ptr<Image_frame> &img_pose )
{







 





    cv::Mat new_gray ;
    cv::Mat old_gray ;
    m_old_gray.convertTo(old_gray, CV_32F);
    curr_img.convertTo(new_gray, CV_32F);

    old_gray = old_gray/255 ;
    new_gray = new_gray/255 ; 
    

    cv::Mat original_new_gray ; 
    new_gray.copyTo(original_new_gray);

    std::vector<int> scales = {10 ,9 , 7 , 5 , 3 ,1 };
    std::vector<int> iter = {20,20 , 20,20 , 0 , 0 }  ;
    std::vector<int> kernel_size = {5,5 , 5,5 , 0 , 0 } ;
    
    cv::Mat deform_row  ; // = cv::Mat::zeros(new_gray.rows, new_gray.cols, CV_32F) ; 
    cv::Mat deform_col  ; //= cv::Mat::zeros(new_gray.rows, new_gray.cols, CV_32F) ; 
    cv::Mat deform_row_before ;
    cv::Mat deform_col_before ;
    cv::Mat diffrence ;
    for(int i = 0 ; i<scales.size() ; i ++)
    {
        cv::Mat curr_deform_row ; 
        cv::Mat curr_deform_col ; 
        if(i==0)
        {
        
           deform_row_before = cv::Mat::zeros( new_gray.rows / scales[i] , new_gray.cols /scales[i], CV_32F) ; 
           deform_col_before  = cv::Mat::zeros( new_gray.rows / scales[i] ,new_gray.cols /scales[i], CV_32F) ; 
            demon_scale(old_gray , new_gray , curr_deform_row , curr_deform_col ,  scales[i]  , iter[i] , kernel_size[i]  , deform_row_before  , deform_col_before ) ;
        }
        else 
        {

            demon_scale(old_gray , new_gray , curr_deform_row , curr_deform_col ,  scales[i]  , iter[i] , kernel_size[i]  , deform_row_before  , deform_col_before ) ;
        }
        curr_deform_row.copyTo(deform_row_before) ;
        curr_deform_col.copyTo(deform_col_before) ;
        



    }
    deform_row = deform_row_before ; 
    deform_col = deform_col_before ; 





    
     curr_tracked_pts.clear()  ;
     cv::Mat grayscale ; 
        new_gray.convertTo(grayscale, CV_8UC1, 255.0);
       cv::Mat new_gray_rgb(new_gray.size(), CV_8UC3) ; 
       cv::cvtColor(grayscale, new_gray_rgb, cv::COLOR_GRAY2RGB);





       cv::Mat grayscale1 ; 
        old_gray.convertTo(grayscale1, CV_8UC1, 255.0);
       cv::Mat old_gray_rgb(new_gray.size(), CV_8UC3) ; 
       cv::cvtColor(grayscale1, old_gray_rgb, cv::COLOR_GRAY2RGB);


      //cv::Mat old_gray_rgb(old_gray.size(), CV_8UC3) ; 
      //cv::cvtColor(old_gray*255, new_gray_rgb, cv::COLOR_GRAY2RGB);
    //std::cout <<last_tracked_pts.size() -m_rgb_pts_ptr_vec_in_last_frame.size() << std::endl;
     for(int i = 0 ; i<last_tracked_pts.size() ; i++ )
     {
        

          RGB_pts *rgb_pts_ptr = ( ( RGB_pts * ) m_rgb_pts_ptr_vec_in_last_frame[ m_old_ids[ i ] ] );
          double depth = (rgb_pts_ptr->get_pos() -  img_pose->m_pose_w2c_t ).norm() ; 
         Eigen::Vector3i jet  = jetColorMap2(depth/3.0) ; 
        cv::Vec3b pixel(jet[2], jet[1], jet[0]);




         
        int row = round(last_tracked_pts[i].y) ; 
        int col = round(last_tracked_pts[i].x) ; 
        old_gray_rgb.at<cv::Vec3b>(row, col) = pixel ;
        int new_row = round(row - deform_row.at<float>(row,col) ) ; 
        int new_col = round(col- deform_col.at<float>(row,col) ) ;
        cv::Point2f point  ; 
        point.y = last_tracked_pts[i].y  - deform_row.at<float>(row,col) ; 
        point.x =last_tracked_pts[i].x - deform_col.at<float>(row,col) ; 
        if(new_row > new_gray_rgb.rows - 1 || new_row < 0 || new_col < 0 || new_col > new_gray_rgb.cols -1   ) 
        {
             status.push_back(0) ; 
             curr_tracked_pts.push_back(point ) ;
            continue ; 
        }
        new_gray_rgb.at<cv::Vec3b>(new_row, new_col) = pixel ;

        status.push_back(1) ; 
        curr_tracked_pts.push_back(point ) ;




     }

     //cv::imwrite("/app/images_reg/" + std::to_string(m_frame_idx) + "_1" ,old_gray_rgb ) ;
      //cv::imwrite("/app/images_reg/new" + std::to_string(m_frame_idx) + ".png" ,new_gray_rgb) ;
      cv::imwrite("/app/images_reg/" + std::to_string(m_frame_idx) + "old.png" ,old_gray_rgb) ;
      cv::imwrite("/app/images_reg/" + std::to_string(m_frame_idx) + "new.png" ,new_gray_rgb) ;



     //outfile.close() ; 
      //auto endTime = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
     //std::cout << "Execution time: " << duration << " milliseconds" << std::endl;




    // cv::Mat after_solution(new_gray.rows, new_gray.cols, CV_32F) ;  //(new_gray.rows, new_gray.cols, CV_32F, cv::Scalar(0));
    // for (int i = 0; i < after_solution.rows; i++)
    //  {
    //      for (int j = 0; j < after_solution.cols; j++)
    //     {

            
    //         //float  new_gray_value  =  get_interpulation_value(old_gray , i ,  j ,   7 , i +  deform_row.at<float>(i,j) ,j +  deform_col.at<float>(i,j)   )  ;
    //         int new_row = round(i - deform_row.at<float>(i,j)) ; 
    //         int new_col =  round(j  - deform_col.at<float>(i,j)) ; 
    //          if(new_row > new_gray_rgb.rows - 1 || new_row < 0 || new_col < 0 || new_col > new_gray_rgb.cols -1   ) 
    //         {
    //             continue ; 
    //         }
    //         after_solution.at<float>(new_row,new_col ) = old_gray.at<float>(i,j) ;
    //     }


    //  }







    //  cv::imwrite("/app/images/" + std::to_string(m_frame_idx) + "before_reg.png" ,255*old_gray ) ;
    //  cv::imwrite("/app/images/" + std::to_string(m_frame_idx) + "after_reg.png" ,255*after_solution ) ;
    //  cv::imwrite("/app/images/" + std::to_string(m_frame_idx) +"to_reg.png" ,255*new_gray ) ;







    
}

void Rgbmap_tracker::track_img(std::shared_ptr<Image_frame> &img_pose, double dis, int if_use_opencv)
{
    Common_tools::Timer tim;
    m_current_frame = img_pose->m_img;
    m_current_frame_time = img_pose->m_timestamp;
    m_map_rgb_pts_in_current_frame_pos.clear();
    if (m_current_frame.empty())
        return;
    cv::Mat frame_gray = img_pose->m_img_gray;
    tim.tic("HE");
    tim.tic("opTrack");
    std::vector<uchar> status;
    std::vector<float> err;
    m_current_tracked_pts = m_last_tracked_pts;
    int before_track = m_last_tracked_pts.size();
    if (m_last_tracked_pts.size() < 30)
    {
        m_last_frame_time = m_current_frame_time;
        return;
    }

    // m_lk_optical_flow_kernel->track_image( frame_gray, m_last_tracked_pts, m_current_tracked_pts, status, 2 );
    // track_image(m_old_gray , frame_gray, m_last_tracked_pts, m_current_tracked_pts, status, 2 );
    //std::cout << m_last_tracked_pts.size() << std::endl;
     demon_track_image(frame_gray, m_last_tracked_pts, m_current_tracked_pts, status, img_pose);


      reduce_vector( m_last_tracked_pts, status );
      reduce_vector( m_old_ids, status );
      reduce_vector( m_current_tracked_pts, status );

     int     after_track = m_last_tracked_pts.size();



     cv::Mat mat_F;

     tim.tic( "Reject_F" );
     unsigned int pts_before_F = m_last_tracked_pts.size();
     mat_F = cv::findFundamentalMat( m_last_tracked_pts, m_current_tracked_pts, cv::FM_RANSAC, 1.0, 0.997, status );
     unsigned int size_a = m_current_tracked_pts.size();
     reduce_vector( m_last_tracked_pts, status );
     reduce_vector( m_old_ids, status );
     reduce_vector( m_current_tracked_pts, status );

    m_map_rgb_pts_in_current_frame_pos.clear();
     double frame_time_diff = ( m_current_frame_time - m_last_frame_time );
      for ( uint i = 0; i < m_last_tracked_pts.size(); i++ )
          {
        if ( img_pose->if_2d_points_available( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y, 1.0, 0.05 ) )
         {
             RGB_pts *rgb_pts_ptr = ( ( RGB_pts * ) m_rgb_pts_ptr_vec_in_last_frame[ m_old_ids[ i ] ] );
             m_map_rgb_pts_in_current_frame_pos[ rgb_pts_ptr ] = m_current_tracked_pts[ i ];
             cv::Point2f pt_img_vel = ( m_current_tracked_pts[ i ] - m_last_tracked_pts[ i ] ) / frame_time_diff;
             rgb_pts_ptr->m_img_pt_in_last_frame = vec_2( m_last_tracked_pts[ i ].x, m_last_tracked_pts[ i ].y );
             rgb_pts_ptr->m_img_pt_in_current_frame = vec_2( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y );
             rgb_pts_ptr->m_img_vel = vec_2( pt_img_vel.x, pt_img_vel.y );
         }
     }

    //  if ( dis > 0 )
    //       {
    //      reject_error_tracking_pts( img_pose, dis );
    //  }



    cv::cvtColor(frame_gray, m_debug_track_img, cv::COLOR_GRAY2BGR);
     for ( uint i = 0; i < m_current_tracked_pts.size(); i++ )
     {
         cv::arrowedLine(m_debug_track_img, m_last_tracked_pts[i], m_current_tracked_pts[i], cv::Scalar(0,0,255),3);
     }





     m_old_gray = frame_gray.clone();
     m_old_frame = m_current_frame;
   // m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos;
    update_last_tracking_vector_and_ids();
    m_frame_idx++;
     m_last_frame_time = m_current_frame_time;
  }

int Rgbmap_tracker::get_all_tracked_pts(std::vector<std::vector<cv::Point2f>> *img_pt_vec)
{
    int hit_count = 0;
    for (auto it = m_map_id_pts_vec.begin(); it != m_map_id_pts_vec.end(); it++)
    {
        if (it->second.size() == m_frame_idx)
        {
            hit_count++;
            if (img_pt_vec)
            {
                img_pt_vec->push_back(it->second);
            }
        }
    }
    cout << "Total frame " << m_frame_idx;
    cout << ", success tracked points = " << hit_count << endl;
    return hit_count;
}

int Rgbmap_tracker::remove_outlier_using_ransac_pnp(std::shared_ptr<Image_frame> &img_pose, int if_remove_ourlier)
{
    Common_tools::Timer tim;
    tim.tic();

    cv::Mat r_vec, t_vec;
    cv::Mat R_mat;
    vec_3 eigen_r_vec, eigen_t_vec;
    std::vector<cv::Point3f> pt_3d_vec, pt_3d_vec_selected;
    std::vector<cv::Point2f> pt_2d_vec, pt_2d_vec_selected;
    std::vector<void *> map_ptr_vec;
    for (auto it = m_map_rgb_pts_in_current_frame_pos.begin(); it != m_map_rgb_pts_in_current_frame_pos.end(); it++)
    {
        map_ptr_vec.push_back(it->first);
        vec_3 pt_3d = ((RGB_pts *)it->first)->get_pos();
        pt_3d_vec.push_back(cv::Point3f(pt_3d(0), pt_3d(1), pt_3d(2)));
        pt_2d_vec.push_back(it->second);
    }
    if (pt_3d_vec.size() < 10)
    {
        return 0;
    }
    if (1)
    {
        std::vector<int> status;
        try
        {
            cv::solvePnPRansac(pt_3d_vec, pt_2d_vec, m_intrinsic, cv::Mat(), r_vec, t_vec, false, 200, 1.5, 0.99,
                               status); // SOLVEPNP_ITERATIVE
        }
        catch (cv::Exception &e)
        {
            scope_color(ANSI_COLOR_RED_BOLD);
            cout << "Catching a cv exception: " << e.msg << endl;
            return 0;
        }
        if (if_remove_ourlier)
        {
            // Remove outlier
            m_map_rgb_pts_in_last_frame_pos.clear();
            m_map_rgb_pts_in_current_frame_pos.clear();
            for (unsigned int i = 0; i < status.size(); i++)
            {
                int inlier_idx = status[i];
                {
                    m_map_rgb_pts_in_last_frame_pos[map_ptr_vec[inlier_idx]] = pt_2d_vec[inlier_idx];
                    m_map_rgb_pts_in_current_frame_pos[map_ptr_vec[inlier_idx]] = pt_2d_vec[inlier_idx];
                }
            }
        }
        update_last_tracking_vector_and_ids();
    }

    // cv::solvePnP(pt_3d_vec, pt_2d_vec, m_intrinsic, m_dist_coeffs * 0, r_vec, t_vec);

    cv::cv2eigen(r_vec, eigen_r_vec);
    cv::cv2eigen(t_vec, eigen_t_vec);
    // eigen_q solver_q = Sophus::SO3d::exp(eigen_r_vec).unit_quaternion().inverse();
    eigen_q solver_q = Sophus::SO3d::exp(eigen_r_vec).unit_quaternion().inverse();
    vec_3 solver_t = (solver_q * eigen_t_vec) * -1.0;
    // cout << "Solve pose: " << solver_q.coeffs().transpose() << " | " << solver_t.transpose() << endl;
    int if_update = 1;
    double t_diff = (solver_t - img_pose->m_pose_w2c_t).norm();
    double r_diff = (solver_q).angularDistance(img_pose->m_pose_w2c_q) * 57.3;

    if_update = 1;
    t_last_estimated = solver_t;
    if (if_update)
    {

        img_pose->m_pnp_pose_w2c_q = solver_q;
        img_pose->m_pnp_pose_w2c_t = solver_t;

        img_pose->m_pose_w2c_q = solver_q;
        img_pose->m_pose_w2c_t = solver_t;
    }
    else
    {
        img_pose->m_pnp_pose_w2c_q = img_pose->m_pose_w2c_q;
        img_pose->m_pnp_pose_w2c_t = img_pose->m_pose_w2c_t;
    }
    img_pose->refresh_pose_for_projection();
    img_pose->m_have_solved_pnp = 1;
    // cout << "Estimate pose cost time = " << tim.toc() << endl;
    return if_update;
}