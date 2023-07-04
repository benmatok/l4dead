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
    double max_allow_repro_err = 2.0 * img_pose->m_img_cols / 320.0;
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
            new_add_pt++;
            if (m_map_rgb_pts_in_last_frame_pos.size() >= m_maximum_vio_tracked_pts)
            {
                break;
            }
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

// Bilinear interpolation function
double bilinearInterpolation(const std::vector<Point>& points, double x, double y)
{
    // Find the four closest sample points
    Point p1, p2, p3, p4;
    bool foundP1 = false, foundP2 = false, foundP3 = false, foundP4 = false;
    for (const auto& point : points)
    {
        if (point.x <= x && point.y <= y)
        {
            p1 = point;
            foundP1 = true;
        }
        else if (point.x > x && point.y <= y)
        {
            p2 = point;
            foundP2 = true;
        }
        else if (point.x <= x && point.y > y)
        {
            p3 = point;
            foundP3 = true;
        }
        else if (point.x > x && point.y > y)
        {
            p4 = point;
            foundP4 = true;
        }
    }

    // Check if any of the required sample points is missing
    if (!(foundP1 && foundP2 && foundP3 && foundP4))
    {
        std::cout <<x << " " << y << std::endl;
        return 0.0;
    }

    // Check if the target point lies within the bounding rectangle
    if (x < p1.x || x > p2.x || y < p1.y || y > p3.y)
    {
        return 0.0;
    }

    // Check if the sample points lie on a horizontal line
    if (p1.y == p2.y && p3.y == p4.y)
    {
        double z1 = p1.z + (x - p1.x) / (p2.x - p1.x) * (p2.z - p1.z);
        double z2 = p3.z + (x - p3.x) / (p4.x - p3.x) * (p4.z - p3.z);
        return z1 + (y - p1.y) / (p3.y - p1.y) * (z2 - z1);
    }



     if (p1.x == p3.x)
    {
        double z1 = p1.z + (y - p1.y) / (p3.y - p1.y) * (p3.z - p1.z);
        double z2 = p2.z + (y - p2.y) / (p4.y - p2.y) * (p4.z - p2.z);
        return z1 + (x - p1.x) / (p2.x - p1.x) * (z2 - z1);
    }


    // Perform bilinear interpolation
    double z1 = p1.z * (p2.x - x) * (p2.y - y);
    double z2 = p2.z * (x - p1.x) * (p2.y - y);
    double z3 = p3.z * (p4.x - x) * (y - p3.y);
    double z4 = p4.z * (x - p3.x) * (y - p3.y);

    return (z1 + z2 + z3 + z4) / ((p2.x - p1.x) * (p3.y - p1.y));
}





double get_interpulation_value(cv::Mat &image , int row , int col ,int kernel_size , double x , double y   ) 
{
    



    if(x >= image.rows  -2 ||  y <= 0 || x <= 0  || y >=image.cols  -2 )
    {
        return image.at<double>(row,col) ; 

    }




    int rows = image.rows ; 
    int cols = image.cols ; 

    int p1_x =  floor(x ) ; 
    int p1_y =  floor(y ) ; 
    double p1_z  =  image.at<double>(p1_x,p1_y) ; 



    int p2_x =  ceil(x+1 ) ; 
    int p2_y =   floor(y ) ; 
    double p2_z  =  image.at<double>(p2_x,p2_y) ;


    int p3_y =  ceil(y+1 ) ; 
    int p3_x =   floor(x ) ; 
    double p3_z  =  image.at<double>(p3_x,p3_y) ;

    int p4_y =  ceil(y+1 ) ; 
    int p4_x =   ceil(x+1 ) ; 

     double p4_z  =  image.at<double>(p4_x,p4_y) ;



    // Check if the sample points lie on a horizontal line
    
    if (p1_y == p2_y && p3_y == p4_y)
    {
        double z1 = p1_z + (x - p1_x) / (p2_x - p1_x) * (p2_z - p1_z);
        double z2 = p3_z + (x - p3_x) / (p4_x - p3_x) * (p4_z - p3_z);
    


        double value = z1 + (y - p1_y) / (p3_y - p1_y) * (z2 - z1);

        return value;
    }



     if (p1_x == p3_x)
    {
        double z1 = p1_z + (y - p1_y) / (p3_y - p1_y) * (p3_z - p1_z);
        double z2 = p2_z + (y - p2_y) / (p4_y - p2_y) * (p4_z - p2_z);
        double value =  z1 + (x - p1_x) / (p2_x - p1_x) * (z2 - z1);



        return value;
    }


    // Perform bilinear interpolation
    double z1 = p1_z * (p2_x - x) * (p2_y - y);
    double z2 = p2_z * (x - p1_x) * (p2_y - y);
    double z3 = p3_z * (p4_x - x) * (y - p3_y);
    double z4 = p4_z * (x - p3_x) * (y - p3_y);

   double value =  (z1 + z2 + z3 + z4) / ((p2_x - p1_x) * (p3_y - p1_y));
    return value ; 



    

 



}




void Rgbmap_tracker::demon_track_image(cv::Mat &curr_img, const std::vector<cv::Point2f> &last_tracked_pts, std::vector<cv::Point2f> &curr_tracked_pts, std::vector<uchar> &status, int opm_method)
{
    std::ofstream outfile;

    outfile.open("/app/debug.txt") ; 
    double diff_intensity = 0 ;


    //curr_img = cv::imread("/catkin_ws/src/r3live/test/moving.png"); 
    //m_old_gray = cv::imread("/catkin_ws/src/r3live/test/static.png");
    
    
    cv::Mat old_gray ;
    m_old_gray.convertTo(old_gray, CV_64F);
    cv::Mat new_gray ;
    curr_img.convertTo(new_gray, CV_64F);
    std::cout << new_gray.rows << std::endl;
    std::cout << new_gray.cols << std::endl;



    cv::Mat x_kernel =  (cv::Mat_<double>(1, 3) << -0.5f, 0.0f, 0.5f);
    cv::Mat grad_x_old ;
    new_gray = new_gray/255 ; 
    old_gray = old_gray/255 ; 


    cv::filter2D(old_gray, grad_x_old, -1, x_kernel); 
     cv::imwrite( "/app/grad.png", old_gray ); 
    cv::Mat grad_y_old  ;
    cv::filter2D(old_gray, grad_y_old, -1, x_kernel.t() );
    //Eigen::MatrixXd deform_row (new_gray.rows, new_gray.cols) ;
    //deform_row.setZero() ; 
    //Eigen::MatrixXd deform_col (new_gray.rows, new_gray.cols) ; 
    //deform_col.setZero() ;  


    cv::Mat deform_row (old_gray.rows, old_gray.cols, CV_64F, cv::Scalar(0));
    cv::Mat deform_col (old_gray.rows, old_gray.cols, CV_64F, cv::Scalar(0));





    
     for (int i = 0; i < new_gray.rows; i++)
     {
         for (int j = 0; j < new_gray.cols; j++)
        {

            // deform_row(i, j) = i;
            // deform_col(i, j) = j;


            //outfile << std::setprecision(10) <<  deform_row(i, j)  << std::endl; 
            double diff =   (old_gray.at<double>(i,j) -  new_gray.at<double>(i,j) ) ; 
            diff_intensity +=  diff*diff ; 
        }


    }

    


    cv::Mat original_new_gray ; 
     new_gray.copyTo(original_new_gray);
    std::cout << "before_intensity"  << diff_intensity <<  std::endl ;  
    for (int iter = 0; iter < 200; iter++)
    {   
        //cv::Mat temp_new_gray ; 
        //new_gray.copyTo(temp_new_gray);
        //Eigen::MatrixXd clone_deform_row = deform_row ; //.replicate(deform_row.rows() , deform_row.cols() ) ; 
        //Eigen::MatrixXd clone_deform_col = deform_col; //.replicate(deform_row. rows() , deform_row.cols() ) ; 
        for (int i = 0; i < new_gray.rows; i++)
        {
            for (int j = 0; j < new_gray.cols; j++)
            {
                 
                

                

                double  new_gray_value  = get_interpulation_value(original_new_gray , i ,  j ,   7 , i +  deform_row.at<double>(i,j) ,j +  deform_col.at<double>(i,j)   )  ;
                double grad_x = grad_x_old.at<double>(i, j);
                double grad_y = grad_y_old.at<double>(i, j);

        
                double i_minus_jt = old_gray.at<double>(i, j) - new_gray_value;
                double grad_norm_2 = grad_x * grad_x + grad_y * grad_y;
                double v_row = 0 ;
                double v_col = 0 ;
                double alpha = 1 ;

                if(grad_norm_2 >0   )
                {
                v_row = i_minus_jt * grad_y / (grad_norm_2 + i_minus_jt * i_minus_jt);
                v_col = i_minus_jt * grad_x/ (grad_norm_2 + i_minus_jt * i_minus_jt);
                }
                deform_row.at<double>(i,j) += v_row ; 
                deform_col.at<double>(i,j) += v_col ; 
                




                

            }
        }

        cv::Size blurSize(7, 7 );  // Kernel size for blurring (adjust as needed)
        cv::Mat blurred_deform_row ; 
        cv::Mat blurred_deform_col ; 
                // Apply the box blur
        cv::boxFilter(deform_row, blurred_deform_row, -1, blurSize);
        cv::boxFilter(deform_col, blurred_deform_col, -1, blurSize);
        blurred_deform_row.copyTo(deform_row);
        blurred_deform_col.copyTo(deform_col);


        
    }
    double after_intensity = 0 ; 
     for (int i = 0; i < new_gray.rows; i++)
     {
         for (int j = 0; j < new_gray.cols; j++)
        {

            // deform_row(i, j) = i;
            // deform_col(i, j) = j;

            double  new_gray_value  = get_interpulation_value(original_new_gray , i ,  j ,   7 , i +  deform_row.at<double>(i,j) ,j +  deform_col.at<double>(i,j)   )  ;
            new_gray.at<double>(i,j) = new_gray_value ;
            double temp = original_new_gray.at<double>(i,j) -new_gray.at<double>(i,j) ; 
            if(temp < 0.01 )
            {
            outfile << std::setprecision(10) <<   deform_row.at<double>(i,j) << " " <<deform_col.at<double>(i,j)   << std::endl;
            get_interpulation_value(original_new_gray , i ,  j ,   0 , i +  deform_row.at<double>(i,j) ,j +  deform_col.at<double>(i,j)   )  ;
            } 
            double diff =   (old_gray.at<double>(i,j) -  new_gray.at<double>(i,j) ) ; 
            after_intensity +=  diff*diff ; 
        }


    }




    cv::imwrite("/app/images/" + std::to_string(m_frame_idx) + "before_reg.png" ,255*original_new_gray ) ;
    cv::imwrite("/app/images/" + std::to_string(m_frame_idx) + "after_reg.png" ,255*new_gray ) ;
    cv::imwrite("/app/images/" + std::to_string(m_frame_idx) +"to_reg.png" ,255*old_gray ) ;
    m_frame_idx+=1;
    std::cout << "after_intensity"  << after_intensity <<  std::endl ; 
    std::cout << "gain_intensity"  << after_intensity - diff_intensity <<  std::endl ; 
    outfile.close();
    m_old_gray = curr_img ;
 
    
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
    demon_track_image(frame_gray, m_last_tracked_pts, m_current_tracked_pts, status, 2);

    // reduce_vector( m_last_tracked_pts, status );
    // reduce_vector( m_old_ids, status );
    // reduce_vector( m_current_tracked_pts, status );

    // int     after_track = m_last_tracked_pts.size();
    // cv::Mat mat_F;

    // tim.tic( "Reject_F" );
    // unsigned int pts_before_F = m_last_tracked_pts.size();
    // mat_F = cv::findFundamentalMat( m_last_tracked_pts, m_current_tracked_pts, cv::FM_RANSAC, 1.0, 0.997, status );
    // unsigned int size_a = m_current_tracked_pts.size();
    // reduce_vector( m_last_tracked_pts, status );
    // reduce_vector( m_old_ids, status );
    // reduce_vector( m_current_tracked_pts, status );

    // m_map_rgb_pts_in_current_frame_pos.clear();
    // double frame_time_diff = ( m_current_frame_time - m_last_frame_time );
    // for ( uint i = 0; i < m_last_tracked_pts.size(); i++ )
    // {
    //     if ( img_pose->if_2d_points_available( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y, 1.0, 0.05 ) )
    //     {
    //         RGB_pts *rgb_pts_ptr = ( ( RGB_pts * ) m_rgb_pts_ptr_vec_in_last_frame[ m_old_ids[ i ] ] );
    //         m_map_rgb_pts_in_current_frame_pos[ rgb_pts_ptr ] = m_current_tracked_pts[ i ];
    //         cv::Point2f pt_img_vel = ( m_current_tracked_pts[ i ] - m_last_tracked_pts[ i ] ) / frame_time_diff;
    //         rgb_pts_ptr->m_img_pt_in_last_frame = vec_2( m_last_tracked_pts[ i ].x, m_last_tracked_pts[ i ].y );
    //         rgb_pts_ptr->m_img_pt_in_current_frame =
    //             vec_2( m_current_tracked_pts[ i ].x, m_current_tracked_pts[ i ].y );
    //         rgb_pts_ptr->m_img_vel = vec_2( pt_img_vel.x, pt_img_vel.y );
    //     }
    // }

    // if ( dis > 0 )
    // {
    //     reject_error_tracking_pts( img_pose, dis );
    // }

    // m_old_gray = frame_gray.clone();
    // //draw debug image with currently tracked points
    // cv::cvtColor(frame_gray, m_debug_track_img, cv::COLOR_GRAY2BGR);
    // for ( uint i = 0; i < m_current_tracked_pts.size(); i++ )
    // {
    //     cv::arrowedLine(m_debug_track_img, m_last_tracked_pts[i], m_current_tracked_pts[i], cv::Scalar(0,0,255),3);
    // }

    // m_old_frame = m_current_frame;
    // m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos;
    // update_last_tracking_vector_and_ids();

    // m_frame_idx++;
    // m_last_frame_time = m_current_frame_time;
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
