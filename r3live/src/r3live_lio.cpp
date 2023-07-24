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
#include "r3live.hpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void R3LIVE::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    double timestamp = msg->header.stamp.toSec();
    if(first_imu_time < -1)
    {
        first_imu_time = timestamp ; 

    }
    //g_camera_lidar_queue.imu_in(timestamp);
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer_lio.clear();
        imu_buffer_vio.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    // Realsense L515 lidar receives IMU data in optical (image) reference frame Needs to be corrected
    if (m_lidar_type == L515)
    {
        double temp_ax = msg->linear_acceleration.x;
        double temp_ay = msg->linear_acceleration.y;
        double temp_az = msg->linear_acceleration.z;
        double temp_gx = msg->angular_velocity.x;
        double temp_gy = msg->angular_velocity.y;
        double temp_gz = msg->angular_velocity.z;
        msg->linear_acceleration.x = temp_az;
        msg->linear_acceleration.y = -temp_ax;
        msg->linear_acceleration.z = -temp_ay;
        msg->angular_velocity.x = temp_gz;
        msg->angular_velocity.y = -temp_gx;
        msg->angular_velocity.z = -temp_gy;
    }

    if (g_camera_lidar_queue.m_if_acc_mul_G)
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }

    imu_buffer_lio.push_back(msg);
    imu_buffer_vio.push_back(msg);

    // std::cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer_lio.size()<<std::endl;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void printf_field_name(sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";
    for (size_t i = 0; i < msg->fields.size(); i++)
    {
        cout << msg->fields[i].name << ", ";
    }
    cout << endl;
}

bool R3LIVE::get_pointcloud_data_from_ros_message(sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZINormal> &pcl_pc)
{

    // printf("Frame [%d] %.3f ", g_LiDAR_frame_index,  msg->header.stamp.toSec() - g_camera_lidar_queue.m_first_imu_time);
    pcl::PointCloud<pcl::PointXYZI> res_pc;
    scope_color(ANSI_COLOR_YELLOW_BOLD);
    // printf_field_name(msg);
    if (msg->fields.size() < 3)
    {
        cout << "Get pointcloud data from ros messages fail!!!" << endl;
        scope_color(ANSI_COLOR_RED_BOLD);
        printf_field_name(msg);
        return false;
    }
    else
    {
        if ((msg->fields.size() == 8) && (msg->fields[3].name == "intensity") &&
            (msg->fields[4].name == "normal_x")) // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg(*msg, pcl_pc);
            return true;
        }
        else if ((msg->fields.size() == 4) && (msg->fields[3].name == "rgb"))
        {
            double maximum_range = 5;
            get_ros_parameter<double>(m_ros_node_handle, "iros_range", maximum_range, 5);
            pcl::PointCloud<pcl::PointXYZRGB> pcl_rgb_pc;
            pcl::fromROSMsg(*msg, pcl_rgb_pc);
            double lidar_point_time = msg->header.stamp.toSec();
            int pt_count = 0;
            pcl_pc.resize(pcl_rgb_pc.points.size());
            for (int i = 0; i < pcl_rgb_pc.size(); i++)
            {
                pcl::PointXYZINormal temp_pt;
                temp_pt.x = pcl_rgb_pc.points[i].x;
                temp_pt.y = pcl_rgb_pc.points[i].y;
                temp_pt.z = pcl_rgb_pc.points[i].z;
                double frame_dis = sqrt(temp_pt.x * temp_pt.x + temp_pt.y * temp_pt.y + temp_pt.z * temp_pt.z);
                if (frame_dis > maximum_range)
                {
                    continue;
                }
                temp_pt.intensity = (pcl_rgb_pc.points[i].r + pcl_rgb_pc.points[i].g + pcl_rgb_pc.points[i].b) / 3.0;
                temp_pt.curvature = 0;
                pcl_pc.points[pt_count] = temp_pt;
                pt_count++;
            }
            pcl_pc.points.resize(pt_count);
            return true;
        }
        else if ((msg->fields.size() == 4) && (msg->fields[3].name == "intensity"))
        {
            double maximum_range = 5;
            get_ros_parameter<double>(m_ros_node_handle, "iros_range", maximum_range, 5);
            pcl::PointCloud<pcl::PointXYZI> pcl_i_pc;
            pcl::fromROSMsg(*msg, pcl_i_pc);
            double lidar_point_time = msg->header.stamp.toSec();
            int pt_count = 0;
            pcl_pc.resize(pcl_i_pc.points.size());
            for (int i = 0; i < pcl_i_pc.size(); i++)
            {
                pcl::PointXYZINormal temp_pt;
                temp_pt.x = pcl_i_pc.points[i].x;
                temp_pt.y = pcl_i_pc.points[i].y;
                temp_pt.z = pcl_i_pc.points[i].z;
                double frame_dis = sqrt(temp_pt.x * temp_pt.x + temp_pt.y * temp_pt.y + temp_pt.z * temp_pt.z);
                if (frame_dis > maximum_range)
                {
                    continue;
                }
                temp_pt.intensity = pcl_i_pc.points[i].intensity;
                temp_pt.curvature = 0;
                pcl_pc.points[pt_count] = temp_pt;
                pt_count++;
            }
            pcl_pc.points.resize(pt_count);
            return true;
        }
        else // TODO, can add by yourself
        {
            cout << "Get pointcloud data from ros messages fail!!! ";
            scope_color(ANSI_COLOR_RED_BOLD);
            printf_field_name(msg);
            return false;
        }
    }
}

bool R3LIVE::sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer_lio.empty())
    {
        return false;
    }

    /*** push lidar frame ***/
    if (!lidar_pushed)
    {
        meas.lidar.reset(new PointCloudXYZINormal());
        if (get_pointcloud_data_from_ros_message(lidar_buffer.front(), *(meas.lidar)) == false)
        {
            return false;
        }
        // pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        meas.lidar_end_time = lidar_end_time;
        // printf("Input LiDAR time = %.3f, %.3f\n", meas.lidar_beg_time, meas.lidar_end_time);
        // printf_line_mem_MB;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer_lio.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer_lio.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer_lio.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time + 0.02)
            break;
        meas.imu.push_back(imu_buffer_lio.front());
        imu_buffer_lio.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // if (meas.imu.empty()) return false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<lidar_end_time<<std::endl;
    return true;
}

// project lidar frame to world
void R3LIVE::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + g_lio_state.pos_ext_i2l) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void R3LIVE::pointBodyToWorldState(PointType const *const pi, PointType *const po, StatesGroup curr_state)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(curr_state.rot_end * (p_body + curr_state.pos_ext_i2l) + curr_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void R3LIVE::RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + g_lio_state.pos_ext_i2l) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}

int R3LIVE::get_cube_index(const int &i, const int &j, const int &k)
{
    return (i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
}

bool R3LIVE::center_in_FOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;

    if (squaredSide1 < 0.4 * cube_len * cube_len)
        return true;

    dis_vec = XAxisPoint_world.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos =
        fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));

    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

bool R3LIVE::if_corner_in_FOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;
    dis_vec = XAxisPoint_world.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;
    float ang_cos =
        fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

void R3LIVE::lasermap_fov_segment()
{
    laserCloudValidNum = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    int centerCubeI = int((g_lio_state.pos_end(0) + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
    int centerCubeJ = int((g_lio_state.pos_end(1) + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
    int centerCubeK = int((g_lio_state.pos_end(2) + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;
    if (g_lio_state.pos_end(0) + 0.5 * cube_len < 0)
        centerCubeI--;
    if (g_lio_state.pos_end(1) + 0.5 * cube_len < 0)
        centerCubeJ--;
    if (g_lio_state.pos_end(2) + 0.5 * cube_len < 0)
        centerCubeK--;
    bool last_inFOV_flag = 0;
    int cube_index = 0;
    cub_needrm.clear();
    cub_needad.clear();
    T2[time_log_counter] = Measures.lidar_beg_time;
    double t_begin = omp_get_wtime();

    while (centerCubeI < FOV_RANGE + 1)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i - 1, j, k)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i - 1, j, k)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - (FOV_RANGE + 1))
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = 0;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i + 1, j, k)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i + 1, j, k)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = laserCloudHeight - 1;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j - 1, k)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j - 1, k)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j + 1, k)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j + 1, k)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = laserCloudDepth - 1;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j, k - 1)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j, k - 1)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j, k + 1)];
                    _last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j, k + 1)];
                }

                featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeK--;
        laserCloudCenDepth--;
    }

    cube_points_add->clear();
    featsFromMap->clear();
    memset(now_inFOV, 0, sizeof(now_inFOV));
    copy_time = omp_get_wtime() - t_begin;
    double fov_check_begin = omp_get_wtime();

    fov_check_time = omp_get_wtime() - fov_check_begin;

    double readd_begin = omp_get_wtime();
#ifdef USE_ikdtree
    if (cub_needrm.size() > 0)
        ikdtree.Delete_Point_Boxes(cub_needrm);
    delete_box_time = omp_get_wtime() - readd_begin;
    // s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if (cub_needad.size() > 0)
        ikdtree.Add_Point_Boxes(cub_needad);
    readd_box_time = omp_get_wtime() - readd_begin - delete_box_time;
    // s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if (cube_points_add->points.size() > 0)
        ikdtree.Add_Points(cube_points_add->points, true);
#endif
    readd_time = omp_get_wtime() - readd_begin - delete_box_time - readd_box_time;
    // s_plot6.push_back(omp_get_wtime() - t_begin);
}

void R3LIVE::feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
{

    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    msg->header.stamp = ros::Time(msg_in->header.stamp.toSec() - m_lidar_imu_time_delay);
    if (g_camera_lidar_queue.lidar_in(msg_in->header.stamp.toSec() + 0.1) == 0)
    {
        return;
    }
    mtx_buffer.lock();
    // std::cout<<"got feature"<<std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    lidar_buffer.push_back(msg);
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void R3LIVE::wait_render_thread_finish()
{
    if (m_render_thread != nullptr)
    {
        m_render_thread->get(); // wait render thread to finish.
        // m_render_thread = nullptr;
    }
}





