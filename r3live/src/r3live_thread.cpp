#include "r3live.hpp"
#include "tools_mem_used.h"
#include "tools_logger.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>




bool R3LIVE::LIO()
{

         nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/world";
    /*** variables definition ***/
        float moving_avg_lio = 0;
        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
        PointCloudXYZINormal::Ptr feats_undistort(new PointCloudXYZINormal());
        PointCloudXYZINormal::Ptr feats_down(new PointCloudXYZINormal());
        PointCloudXYZINormal::Ptr laserCloudOri(new PointCloudXYZINormal());
         PointCloudXYZINormal::Ptr coeffSel(new PointCloudXYZINormal());
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if ( sync_packages( Measures ) == 0 )
            {
                return 0;  
            }
        g_lidar_star_tim = frame_first_pt_time;



        double t0, t1, t2, t3, t4, t5, match_start, match_time, solve_start, solve_time, pca_time, svd_time;
        match_time = 0;

        kdtree_search_time = 0;
        solve_time = 0;
        pca_time = 0;
        svd_time = 0;
        t0 = omp_get_wtime();

        bool imu_need_init = p_imu->Process(Measures, g_lio_state, feats_undistort);





        StatesGroup first_state = g_lio_state;
        StatesGroup state_propagate(g_lio_state);

        // cout << "G_lio_state.last_update_time =  " << std::setprecision(10) << g_lio_state.last_update_time -g_lidar_star_tim  << endl;
        if ( feats_undistort->empty() || ( feats_undistort == NULL ) )
        {
            frame_first_pt_time = Measures.lidar_beg_time;
            std::cout << "not ready for odometry" << std::endl;
            return 0 ;
        } 



        if ((Measures.lidar_beg_time - frame_first_pt_time) < INIT_TIME)
        {
            flg_EKF_inited = false;
            std::cout << "||||||||||Initiallizing LiDAR||||||||||" << std::endl;
        }
        else
        {
            flg_EKF_inited = true;
        }
        /*** Compute the euler angle ***/
        Eigen::Vector3d euler_cur = RotMtoEuler(g_lio_state.rot_end);


        lasermap_fov_segment(); // <1ms
        downSizeFilterSurf.setInputCloud(feats_undistort); // <1ms~
        downSizeFilterSurf.filter(*feats_down); // 3ms~

        //*feats_down = *feats_undistort ;
        // cout <<"Preprocess cost time: " << tim.toc("Preprocess") << endl;
        /*** initialize the map kdtree ***/
        if ((feats_down->points.size() > 1) && (ikdtree.Root_Node == nullptr))
        {
            // std::vector<PointType> points_init = feats_down->points;
            ikdtree.set_downsample_param(filter_size_map_min);
            ikdtree.Build(feats_down->points);
            flg_map_initialized = true;
            return 0 ;
        }

        if (ikdtree.Root_Node == nullptr )
        {
            
            flg_map_initialized = false;
            //std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
            return 0 ;
        }
        int featsFromMapNum = ikdtree.size();

        int feats_down_size = feats_down->points.size();

        /*** ICP and iterated Kalman filter update ***/
        PointCloudXYZINormal::Ptr coeffSel_tmpt(new PointCloudXYZINormal(*feats_down));
        PointCloudXYZINormal::Ptr feats_down_updated(new PointCloudXYZINormal(*feats_down));
        std::vector<double> res_last(feats_down_size, 1000.0);
        std::vector<double> res_not_abs(feats_down_size, 1000.0); // initial
 
        if (featsFromMapNum >= 5)
        {
            t1 = omp_get_wtime();

            if (m_if_publish_feature_map)
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;

                sensor_msgs::PointCloud2 laserCloudMap;
                pcl::toROSMsg(*featsFromMap, laserCloudMap);
                laserCloudMap.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
                // laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
                laserCloudMap.header.frame_id = "world";
                pubLaserCloudMap.publish(laserCloudMap);
            }

            std::vector<bool> point_selected_surf(feats_down_size, true);
            std::vector<std::vector<int>> pointSearchInd_surf(feats_down_size);
            std::vector<PointVector> Nearest_Points(feats_down_size);

            int rematch_num = 0;
            bool rematch_en = 0;
            flg_EKF_converged = 0;
            deltaR = 0.0;
            deltaT = 0.0;
            t2 = omp_get_wtime();
            double maximum_pt_range = 0.0;
            // cout <<"Preprocess 2 cost time: " << tim.toc("Preprocess") << endl;


                
                match_start = omp_get_wtime();
                laserCloudOri->clear();
                coeffSel->clear();
                std::vector<PointType> surface_points;
                std::vector<PointType> surface_points_normals;
                match_start = omp_get_wtime();
                laserCloudOri->clear();
                coeffSel->clear();
                /** closest surface search and residual computation **/
                for (int i = 0; i < feats_down_size; i += m_lio_update_point_step)
                {

                    double search_start = omp_get_wtime();
                    PointType &pointOri_tmpt = feats_down->points[i];
                    double ori_pt_dis =
                        sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
                    maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range);
                    PointType &pointSel_tmpt = feats_down_updated->points[i];

                    /* transform to world frame */
                    pointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt);
                    std::vector<float> pointSearchSqDis_surf;

                    auto &points_near = Nearest_Points[i];

                    point_selected_surf[i] = true;
                    /** Find the closest surfaces in the map **/
                    ikdtree.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                    float max_distance = pointSearchSqDis_surf[NUM_MATCH_POINTS - 1];
                    //  max_distance to add residuals
                    // ANCHOR - Long range pt stragetry
                    if (max_distance > m_maximum_pt_kdtree_dis)
                    {
                        point_selected_surf[i] = false;
                    }

                    kdtree_search_time += omp_get_wtime() - search_start;
                    if (point_selected_surf[i] == false)
                        continue;

                    // match_time += omp_get_wtime() - match_start;
                    double pca_start = omp_get_wtime();
                    /// PCA (using minimum square method)
                    cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
                    cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
                    // cv::Mat matX0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( 0 ) );
                    cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

                    for (int j = 0; j < NUM_MATCH_POINTS; j++)
                    {
                        matA0.at<float>(j, 0) = points_near[j].x;
                        matA0.at<float>(j, 1) = points_near[j].y;
                        matA0.at<float>(j, 2) = points_near[j].z;
                    }

                    cv::solve(matA0, matB0, matX0, cv::DECOMP_QR); // TODO

                    float pa = matX0.at<float>(0, 0);
                    float pb = matX0.at<float>(1, 0);
                    float pc = matX0.at<float>(2, 0);
                    float pd = 1;

                    float ps = sqrt(pa * pa + pb * pb + pc * pc);
                    pa /= ps;
                    pb /= ps;
                    pc /= ps;
                    pd /= ps;

                    bool planeValid = true;
                    for (int j = 0; j < NUM_MATCH_POINTS; j++)
                    {
                        // ANCHOR -  Planar check
                        if (fabs(pa * points_near[j].x + pb * points_near[j].y + pc * points_near[j].z + pd) >
                            m_planar_check_dis) // Raw 0.05
                        {
                            // ANCHOR - Far distance pt processing
                            if (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < m_long_rang_pt_dis))
                            // if(1)
                            {
                                planeValid = false;
                                point_selected_surf[i] = false;
                                break;
                            }
                        }
                    }

                    if (planeValid)
                    {
                        float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                        float s = 1 - 0.9 * fabs(pd2) /
                                          sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y +
                                                    pointSel_tmpt.z * pointSel_tmpt.z));
                        // ANCHOR -  Point to plane distance
                        double acc_distance = (ori_pt_dis < m_long_rang_pt_dis) ? m_maximum_res_dis : 1.0;
                        if (pd2 < acc_distance)
                        {
                            // if(std::abs(pd2) > 5 * res_mean_last)
                            // {
                            //     point_selected_surf[i] = false;
                            //     res_last[i] = 0.0;
                            //     continue;
                            // }

                            surface_points.push_back(pointOri_tmpt);

                            point_selected_surf[i] = true;
                            coeffSel_tmpt->points[i].x = pa;
                            coeffSel_tmpt->points[i].y = pb;
                            coeffSel_tmpt->points[i].z = pc;
                            coeffSel_tmpt->points[i].intensity = pd;
                            res_last[i] = std::abs(pd2);
                            res_not_abs[i] = pd2;
                            surface_points_normals.push_back(coeffSel_tmpt->points[i]);
                        }
                        else
                        {
                            point_selected_surf[i] = false;
                        }
                    }
                    pca_time += omp_get_wtime() - pca_start;
                }
            
                


               
                std::vector<int> best_agree_points;
                double best_num_agree = 0;
                std::chrono::steady_clock::time_point start_ransac = std::chrono::steady_clock::now();



                for (int ransac_iter = 0; ransac_iter < 100; ransac_iter++)
                {
                    if (!flg_EKF_inited)
                    {



                        break ; 
                    }

                    StatesGroup ransac_state  = g_lio_state;

                    std::set<int> selected_ind;

                    while (selected_ind.size() < 3)
                    {
                        int ind = 1 + std::rand() / ((RAND_MAX + 1u) / surface_points.size());


                        selected_ind.insert(ind);
                    }
                    //until converge
                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    
   

                    for( int  j = 0 ;j< 20 ; j++)
                    {
                        Eigen::MatrixXd Hsub(3, 6);
                        Eigen::VectorXd meas_vec(3);
                        Hsub.setZero();
                        
                        int count = 0 ;

                        for (int i : selected_ind)
                        {

                            PointType laser_p  ; 
                            pointBodyToWorldState(&surface_points[i] ,&laser_p , ransac_state  ) ; 
                            Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
                            point_this += ransac_state.pos_ext_i2l;
                            Eigen::Matrix3d point_crossmat;
                            point_crossmat << SKEW_SYM_MATRIX(point_this);

                            /*** get the normal vector of closest surface/corner ***/
                            const PointType &norm_p = surface_points_normals[i];
                            Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
                            
                            /*** calculate the Measuremnt Jacobian matrix H ***/
                            Eigen::Vector3d A(point_crossmat * ransac_state.rot_end.transpose() * norm_vec);
                            Hsub.row(count) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                            /*** Measuremnt: distance to the closest surface/corner ***/
                            double res  = norm_p.x * laser_p.x + norm_p.y * laser_p.y + norm_p.z * laser_p.z + norm_p.intensity;  
                            //res = std::abs(res) ; 
                            meas_vec(count) = -1*res;
                            count++ ; 
                        }
                        Eigen::MatrixXd A = Hsub.transpose() * Hsub ;
                        Eigen::MatrixXd b = Hsub.transpose() *meas_vec  ;
                        Eigen::LDLT<Eigen::MatrixXd> L  = A.ldlt() ;
                        Eigen::VectorXd x = L.solve(b);
                        Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
                        solution.setZero() ; 

                        for(int index = 0 ; index< 6 ; index++)
                        {
                            solution(index , 0 ) = x(index) ;  
                        }

                        ransac_state =ransac_state + solution ; 

                    }
                    // after converge
                    std::vector<int> agree_points;
                    double num_agree = 0;
                    // check how many points agree
                    for (int i = 0; i < surface_points.size(); i++)
                    {


                        PointType laser_p  ; 
                        pointBodyToWorldState(&surface_points[i] ,&laser_p ,  ransac_state  ) ;
                        const PointType &norm_p = surface_points_normals[i];
                        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z); 
                        double res = laser_p.x * surface_points_normals[i].x + laser_p.y * surface_points_normals[i].y + surface_points_normals[i].z * laser_p.z + norm_p.intensity ;
                        if (std::abs(res) < 0.05)
                        {
                            num_agree++;
                            agree_points.push_back(i);
                        }
                    }

                    if (best_num_agree < num_agree)
                    {
                        best_agree_points.clear();
                        best_agree_points = agree_points;
                        best_num_agree = num_agree;
                    }
    
                }
                std::chrono::steady_clock::time_point end_ransac = std::chrono::steady_clock::now();



                 StatesGroup try_update = g_lio_state ; 
                for( int j = 0 ; j<20 ; j++)
                {
                    Eigen::MatrixXd Hsub(best_agree_points.size(), 6);
                    Eigen::VectorXd meas_vec(best_agree_points.size());
                    Hsub.setZero();
                    for (int index = 0; index < best_agree_points.size(); index++)
                    {
                        int i = best_agree_points[index];
                        PointType laser_p ; 
                        pointBodyToWorldState(&surface_points[i] , &laser_p ,try_update  ) ; 
                        Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
                        point_this += try_update.pos_ext_i2l;
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat << SKEW_SYM_MATRIX(point_this);

                        /*** get the normal vector of closest surface/corner ***/
                        const PointType &norm_p = surface_points_normals[i];
                        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        Eigen::Vector3d A(point_crossmat * try_update.rot_end.transpose() * norm_vec);
                        Hsub.row(index) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        double res  = norm_p.x * laser_p.x + norm_p.y * laser_p.y + norm_p.z * laser_p.z + norm_p.intensity  ;  
                        //res = std::abs(res) ; g_lio_state
                        meas_vec(index)  = -1*res;

                    }


                    Eigen::Vector3d rot_add, t_add, v_add, bg_add, ba_add, g_add;
                    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
                    Eigen::MatrixXd K(DIM_OF_STATES, best_agree_points.size());
                    if (!flg_EKF_inited)
                    {
                        cout << ANSI_COLOR_RED_BOLD << "Run EKF init" << ANSI_COLOR_RESET << endl;
                        /*** only run in initialization period ***/
                        set_initial_state_cov(try_update);
                    }
                    else
                    {
                        auto &&Hsub_T = Hsub.transpose();
                        H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;
                        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 = (H_T_H + (try_update.cov / LASER_POINT_COV).inverse()).inverse();

                        K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T;
                        
                        auto vec = state_propagate - try_update;

                        solution = K * (meas_vec - Hsub * vec.block<6, 1>(0, 0));
                        
                        //g_lio_state = state_propagate + solution;

                         //g_lio_state = g_lio_state.normalize_if_large(1);

                         try_update = state_propagate + solution ;
                          try_update = try_update.normalize_if_large(1) ; 
                          StatesGroup resu_states  =  StatesGroup()+ solution ; 



                        if(j==19)
                        {


                        // for(int iter = 0 ; iter < meas_vec.size() ; iter++)
                        // {
                        // outfile<< std::setprecision(9) << meas_vec(iter) << std::endl ; 
                        // }



                       Eigen::Matrix<double, DIM_OF_STATES, 1> diff = try_update - state_propagate;
                       // Eigen::Matrix<double, DIM_OF_STATES, 1> state_resu = resu_states - StatesGroup() ;

                        //vec_3 rot_resu = SO3_LOG(resu_states.rot_end) ;
                        //mat_3_3 rot_cov = state_propagate.cov.block(0, 0, 3, 3) ; 
                        double mahalanobis_distance = std::sqrt(diff.transpose() *state_propagate.cov.inverse() * diff ) ; 
                        std::ofstream outfile_mah("/app/mahalanobis_distance.txt", std::ios::app);
                        outfile_mah <<  std::setprecision(16) << try_update.last_update_time << " "<<mahalanobis_distance << std::endl ; 
                        outfile_mah.close() ;
                        if(mahalanobis_distance<6)
                        {
                             g_lio_state = try_update ; 
                               g_lio_state.last_update_time = Measures.lidar_end_time;
                             euler_cur = RotMtoEuler( g_lio_state.rot_end );
                             dump_lio_state_to_log( m_lio_state_fp );
                             G.block<DIM_OF_STATES, 6>(0, 0) = K * Hsub;
                            g_lio_state.cov = (I_STATE - G) * g_lio_state.cov;
                             position_last = g_lio_state.pos_end;
                           solve_time += omp_get_wtime() - solve_start;

                        }





                        }
                    }
                
            }





            

            t3 = omp_get_wtime();
            /*** add new frame points to map ikdtree ***/
            PointVector points_history;
            ikdtree.acquire_removed_points( points_history );
        
            memset( cube_updated, 0, sizeof( cube_updated ) );

            for ( int i = 0; i < points_history.size(); i++ )
            {
                PointType &pointSel = points_history[ i ];

                int cubeI = int( ( pointSel.x + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
                int cubeJ = int( ( pointSel.y + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
                int cubeK = int( ( pointSel.z + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;

                if ( pointSel.x + 0.5 * cube_len < 0 )
                    cubeI--;
                if ( pointSel.y + 0.5 * cube_len < 0 )
                    cubeJ--;
                if ( pointSel.z + 0.5 * cube_len < 0 )
                    cubeK--;

                if ( cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth )
                {
                    int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                    featsArray[ cubeInd ]->push_back( pointSel );
                }
            }

            for ( int i = 0; i < feats_down_size; i++ )
            {
                /* transform to world frame */
                pointBodyToWorld( &( feats_down->points[ i ] ), &( feats_down_updated->points[ i ] ) );
            }
            t4 = omp_get_wtime();
            
            ikdtree.Add_Points( feats_down_updated->points, true );
            
            kdtree_incremental_time = omp_get_wtime() - t4 + readd_time + readd_box_time + delete_box_time;
            t5 = omp_get_wtime();


 
            /******* Publish current frame points in world coordinates:  *******/
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? (*feats_undistort) : (*feats_down);

            int laserCloudFullResNum = laserCloudFullRes2->points.size();
            lidar_time  = Measures.lidar_beg_time ; 
            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            {
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    RGBpointBodyToWorld(&laserCloudFullRes2->points[i], &temp_point);
                    laserCloudFullResColor->push_back(temp_point);
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
                // laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.stamp.fromSec(Measures.lidar_end_time);
                laserCloudFullRes3.header.frame_id = "world"; // world; camera_init
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }

            if ( 1) // append point cloud to global map.
            {
                static std::vector<double> stastic_cost_time;
                Common_tools::Timer tim;
                // tim.tic();
                // ANCHOR - RGB maps update
                wait_render_thread_finish();
                    m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(
                        *laserCloudFullResColor, Measures.lidar_end_time -first_imu_time, nullptr,
                        m_append_global_map_point_step);
                stastic_cost_time.push_back(tim.toc(" ", 0));
            }

            /******* Publish Maps:  *******/
            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg(*featsFromMap, laserCloudMap);
            laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
            laserCloudMap.header.frame_id = "world";
            pubLaserCloudMap.publish(laserCloudMap);

            /******* Publish Odometry ******/
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
            odomAftMapped.header.frame_id = "world";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
            odomAftMapped.pose.pose.orientation.x = geoQuat.x;
            odomAftMapped.pose.pose.orientation.y = geoQuat.y;
            odomAftMapped.pose.pose.orientation.z = geoQuat.z;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = g_lio_state.pos_end(0);
            odomAftMapped.pose.pose.position.y = g_lio_state.pos_end(1);
            odomAftMapped.pose.pose.position.z = g_lio_state.pos_end(2);

            pubOdomAftMapped.publish(odomAftMapped);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(
                tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z));
            q.setW(odomAftMapped.pose.pose.orientation.w);
            q.setX(odomAftMapped.pose.pose.orientation.x);
            q.setY(odomAftMapped.pose.pose.orientation.y);
            q.setZ(odomAftMapped.pose.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(Measures.lidar_end_time), "world", "/aft_mapped"));

            msg_body_pose.header.stamp = ros::Time::now();
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = g_lio_state.pos_end(0);
            msg_body_pose.pose.position.y = g_lio_state.pos_end(1);
            msg_body_pose.pose.position.z = g_lio_state.pos_end(2);
            msg_body_pose.pose.orientation.x = geoQuat.x;
            msg_body_pose.pose.orientation.y = geoQuat.y;
            msg_body_pose.pose.orientation.z = geoQuat.z;
            msg_body_pose.pose.orientation.w = geoQuat.w;


            


            /******* Publish Path ********/
            msg_body_pose.header.frame_id = "world";
            if (frame_num > 10)
            {
                path.poses.push_back(msg_body_pose);
            }
            pubPath.publish(path);

            /*** save debug variables ***/
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            // aver_time_consu = aver_time_consu * 0.8 + (t5 - t0) * 0.2;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = aver_time_consu;
            s_plot2[time_log_counter] = kdtree_incremental_time;
            s_plot3[time_log_counter] = kdtree_search_time;
            s_plot4[time_log_counter] = fov_check_time;
            s_plot5[time_log_counter] = t5 - t0;
            s_plot6[time_log_counter] = readd_box_time;
        }
        g_LiDAR_frame_index++ ; 
    return 1;

}


bool R3LIVE::VIO()
{

        std::cout << "hello" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        Common_tools::Timer tim;
        std::shared_ptr<Image_frame> img_pose ; 
        //m_queue_image_with_pose.try_pop(img_pose);
        double message_time =0;
        while(  lidar_time - message_time >0.01   )
        {
            if(!m_queue_image_with_pose.empty() )
            {
                m_queue_image_with_pose.try_pop(img_pose);
               message_time = img_pose->m_timestamp;
            }

        }
        



         if(message_time -lidar_time  > 0.01  )
         {

            return 0 ; 
         }
        img_pose->set_frame_idx(g_camera_frame_idx);

        if (g_camera_frame_idx == 0)
        {

            std::vector<cv::Point2f> pts_2d_vec;
            std::vector<std::shared_ptr<RGB_pts>> rgb_pts_vec;


            set_image_pose(img_pose, g_lio_state); // For first frame pose, we suppose that the motion is static.
            m_map_rgb_pts.selection_points_for_projection(img_pose, &rgb_pts_vec, &pts_2d_vec, 0.01);
            op_track.init(img_pose, rgb_pts_vec, pts_2d_vec);
            g_camera_frame_idx++;
            return 1;
            last_timestamp = img_pose->m_timestamp ; 
            last_t_vec[0] = 0 ;
            last_t_vec[1] = 0 ;
            last_t_vec[2] = 0 ;

        }
        g_camera_frame_idx++;


        StatesGroup state_out;
        m_cam_measurement_weight = std::max(0.001, std::min(5.0 / m_number_of_new_visited_voxel, 0.01));
        state_out = g_lio_state;
        if (vio_preintegration(g_lio_state, state_out, img_pose->m_timestamp + g_lio_state.td_ext_i2c) == false )
        {
            return 0 ; 
        }
        
        
    
        set_image_pose(img_pose, state_out);
         //laser_to_camera(laserCloudFullRes2 ,img_pose,  g_lio_state , counter , lidar_time , outfile ); 
         //counter++ ; 

        op_track.track_img(img_pose, -20);
         g_cost_time_logger.record(tim, "Track_img");
        // cout << "Track_img cost " << tim.toc( "Track_img" ) << endl;
         tim.tic("Ransac");
        //set_image_pose( img_pose, state_out );
        //  ANCHOR -  remove point using PnP.
        cv::Mat r_vec ;
        cv::Mat t_vec ;
        vec_3 eigen_r_vec, eigen_t_vec;
        
        if (op_track.remove_outlier_using_ransac_pnp( r_vec, t_vec , img_pose   ) == 0)
        {

            cout << ANSI_COLOR_RED_BOLD << "****** Remove_outlier_using_ransac_pnp error*****" << ANSI_COLOR_RESET << endl;
        }
        
        else{
            cv::cv2eigen(t_vec, eigen_t_vec);
            cv::cv2eigen(r_vec, eigen_r_vec);
            vec_3 delta_p = eigen_t_vec - last_t_vec ;
            double delta_time =  img_pose->m_timestamp - last_timestamp;
            double velocity_norm  = (delta_p *(1.0/(delta_time+1e-6))).norm() ; 
            double state_vel_norm = state_out.vel_end.norm() ; 
            if(velocity_norm < state_vel_norm )
            {
                state_out.vel_end = (velocity_norm / state_vel_norm) *state_out.vel_end ;

            }
            last_timestamp = img_pose->m_timestamp ; 
            last_t_vec = eigen_t_vec ;

            




        }
        g_cost_time_logger.record(tim, "Ransac");
         tim.tic("Vio_f2f");
        bool res_esikf = true, res_photometric = true;
        wait_render_thread_finish();
        res_esikf = vio_esikf(state_out, op_track);
        g_cost_time_logger.record(tim, "Vio_f2f");
        tim.tic("Vio_f2m");
        res_photometric = vio_photometric(state_out, op_track, img_pose);
        g_cost_time_logger.record(tim, "Vio_f2m");


        //auto diff = try_update - state_propagate;                
        //StatesGroup resu_states  =  StatesGroup()+ diff ; 
        //vec_3 rot_resu = SO3_LOG(resu_states.rot_end) ;
        //mat_3_3 rot_cov = state_propagate.cov.block(0, 0, 3, 3) ; 

        //double mahalanobis_distance = std::sqrt(rot_resu.transpose() *rot_cov.inverse() * rot_resu ) ; 
        //std::ofstream outfile_mah("/app/mahalanobis_distance_vio                                                .txt", std::ios::app);
        //outfile_mah <<  std::setprecision(16) << try_update.last_update_time << " "<<mahalanobis_distance << std::endl ; 
        //outfile_mah.close() ;

    
        g_lio_state = state_out;
        set_image_pose( img_pose, g_lio_state );
        print_dash_board();

        if (1)
        {
            tim.tic("Render");
            // m_map_rgb_pts.render_pts_in_voxels(img_pose, m_last_added_rgb_pts_vec);
            if (1) // Using multiple threads for rendering
            {
                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
                // m_map_rgb_pts.render_pts_in_voxels_mp(img_pose, &m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                // img_pose->m_timestamp);
                m_render_thread = std::make_shared<std::shared_future<void>>(m_thread_pool_ptr->commit_task(
                    render_pts_in_voxels_mp, img_pose, &m_map_rgb_pts.m_voxels_recent_visited, img_pose->m_timestamp));
            }
            else
            {
                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
                // m_map_rgb_pts.render_pts_in_voxels( img_pose, m_map_rgb_pts.m_rgb_pts_in_recent_visited_voxels,
                // img_pose->m_timestamp );
            }
            m_map_rgb_pts.m_last_updated_frame_idx = img_pose->m_frame_idx;
            g_cost_time_logger.record(tim, "Render");

            tim.tic("Mvs_record");
            if (m_if_record_mvs)
            {
                // m_mvs_recorder.insert_image_and_pts( img_pose, m_map_rgb_pts.m_voxels_recent_visited );
                m_mvs_recorder.insert_image_and_pts(img_pose, m_map_rgb_pts.m_pts_last_hitted);
            }
            g_cost_time_logger.record(tim, "Mvs_record");
        }
        // ANCHOR - render point cloud
        dump_lio_state_to_log(m_lio_state_fp);


        // cout << "Solve image pose cost " << tim.toc("Solve_pose") << endl;
        m_map_rgb_pts.update_pose_for_projection(img_pose, -0.4);
        op_track.update_and_append_track_pts(img_pose, m_map_rgb_pts, m_track_windows_size / m_vio_scale_factor, 1000000);
        g_cost_time_logger.record(tim, "Frame");
        double frame_cost = tim.toc("Frame");
        g_image_vec.push_back(img_pose);
        frame_cost_time_vec.push_back(frame_cost);
        if (g_image_vec.size() > 10)
        {
            g_image_vec.pop_front();
            frame_cost_time_vec.pop_front();
        }



        double display_cost_time = std::accumulate(frame_cost_time_vec.begin(), frame_cost_time_vec.end(), 0.0) / frame_cost_time_vec.size();
        g_vio_frame_cost_time = display_cost_time;
        // publish_render_pts( m_pub_render_rgb_pts, m_map_rgb_pts );
        publish_camera_odom(img_pose, message_time);
        publish_track_img(op_track.m_debug_track_img, display_cost_time);
        // publish_track_img( img_pose->m_raw_img, display_cost_time );

        if (m_if_pub_raw_img)
        {
            publish_raw_img(img_pose->m_raw_img);
        }
        return 1;



}

void R3LIVE::single_thread()
{
     op_track.set_intrinsic( g_cam_K, g_cam_dist * 0, cv::Size( m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor ) );
    op_track.m_maximum_vio_tracked_pts = m_maximum_vio_tracked_pts;
    m_map_rgb_pts.m_minimum_depth_for_projection = m_tracker_minimum_depth;
    m_map_rgb_pts.m_maximum_depth_for_projection = m_tracker_maximum_depth;
    cv::imshow( "Control panel", generate_control_panel_img().clone() );
    std::ofstream outfile_vec ;
    outfile_vec.open("/app/mahalanobis_distance.txt");
    outfile_vec.close() ; 





    bool succses_first_lio = 0 ; 
    int frame_number = 0 ;
    while (ros::ok())
    {



        if(!vio_or_lio.empty() )
        {
        int msg_type ; 
        vio_or_lio.try_pop(msg_type) ; 
        if(msg_type == 1   )
        {
        std::cout << "start lio " << std::endl;
        succses_first_lio = LIO() ; 
        std::cout << "finish lio"  << std::endl;
        }

        if(msg_type == 0 && succses_first_lio )
        {
            std::cout << "start VIO " << std::endl;
            VIO() ; 
            std::cout << "end VIO " << std::endl;

        }

        }
        
  




    }

}





