import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Vector3
from pathlib import Path
import numpy as np
import pandas as pd
import argparse
import cv2
import os

# data fields that define point cloud message, all fields are float32 type, 4 bytes long
PC_FIELDS = [
    PointField(name='x',offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y',offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z',offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='reflectivity',offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='offset_time',offset=16, datatype=PointField.FLOAT32, count=1)
    ]

def load_scan(scan_path: Path) -> 'tuple[float, np.ndarray]':
    '''method gets path of scan file, loads data and returns numpy array with timestamp'''
    scan = np.loadtxt(scan_path)
    timestamp = float(scan_path.stem)
    return timestamp, scan

def phone2bag(input_dir:Path, output_dir:Path):
    depth_paths=[]
    for path, _, files in os.walk(os.path.join(input_dir,'depthData')):
        for file in files:
            depth_paths.append(Path(os.path.join(path,file)))
    img_paths=[]
    for path, _, files in os.walk(os.path.join(input_dir,'colorImageData')):
        for file in files:
            img_paths.append(Path(os.path.join(path,file)))

    imgs = []
    img_timestamps = []
    for img_path in img_paths:
        img = cv2.imread(str(img_path))
        imgs.append(img)
        timestamp = int(img_path.stem.split('_')[1])
        img_timestamps.append(timestamp)
    imgs = [img for _, img in sorted(zip(img_timestamps,imgs))]
    img_timestamps = sorted(img_timestamps)

    point_clouds=[]
    # TODO: confirm that FOV angle is 79.52 degrees or calibrate camera
    # data from source: main camera has a lens equivelant to a 26mm lens (for a fullframe sensor camera)
    # image size is 240x320
    w_px = 320
    h_px = 240
    f_px = 26*(w_px/36) # 26mm * 320px

    img_x_coords, img_y_coords = np.meshgrid(np.arange(-120,120),np.arange(-160,160))
    img_x_coords = img_x_coords.flatten()
    img_y_coords = img_y_coords.flatten()
    point_cloud_timestamps=[]
    for depth_path in depth_paths:
        with open(depth_path,'r') as f:
            depth_img = np.fromstring(f.read()[1:-1], dtype=int, sep=', ') # assumes all data in file is in the first line

            point_cloud = np.zeros((len(depth_img),5))
            point_cloud[:,2] = depth_img / 1000
            point_cloud[:,1] = img_y_coords * point_cloud[:,2] / f_px
            point_cloud[:,0] = img_x_coords * point_cloud[:,2] / f_px


            # add intensity/reflectivity feature to each pointcloud
            neighbor_i = img_timestamps.searchsorted(timestamp)
            if abs(img_timestamps[neighbor_i] - timestamp) > abs(img_timestamps[neighbor_i+1] - timestamp):
                neighbor_i = neighbor_i+1
            point_cloud[:,3] = cv2.cvtColor(imgs[neighbor_i], code=cv2.COLOR_BGR2GRAY).flatten()
            
            # add point readout time
            # TODO: make sure read speed is right
            # line read speed in CMOS is 10 microseconds (10K nanoseconds)
            np.repeat()
            point_cloud[:,4] = np.tile(np.arange(0,10000*h_px,10000))
            
            point_clouds.append(point_cloud)
            
        point_cloud_timestamps.append(int(depth_path.stem.split('_')[1]))
    point_clouds=[depth_img for _, depth_img in sorted(zip(point_cloud_timestamps, point_clouds))]
    point_cloud_timestamps = sorted(point_cloud_timestamps)

    gyro_data = pd.read_csv(os.path.join(input_dir,'gyroscope.csv'),names=['gyro_x','gyro_y','gyro_z','Timestamp'], index_col='Timestamp')
    acc_data = pd.read_csv(os.path.join(input_dir,'accelerometer.csv'),names=['acc_x','acc_y','acc_z','Timestamp'], index_col='Timestamp')
    imu_data = acc_data.join(gyro_data, how='outer')
    imu_data['Timestamp']=imu_data.index
    imu_data.set_index(pd.to_datetime(imu_data['Timestamp'],unit='ns'), inplace=True)
    imu_data.interpolate(method='time',inplace=True)
    imu_data.set_index('Timestamp', inplace=True)


    with rosbag.Bag(os.path.join(output_dir,input_dir.stem+'.bag'), 'w') as bag:

        # insert color image data
        bridge = CvBridge()
        for timestamp, cv_img in zip(img_timestamps, imgs):
            header = rospy.Header()
            header.stamp = rospy.Time(nsecs=timestamp)
            header.frame_id = 'custom_frame_camera'
            img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            img_msg.header = header
            bag.write('/camera/image_color', img_msg, t=header.stamp)  
        
        # insert point cloud data
        for timestamp, point_cloud in zip(point_cloud_timestamps,point_clouds):
            '''
            Pointcloud2:
            * The Header message contains two main fields:
            -  stamp: Time object representing the time when the message was created
            -  frame_id: string representing the coordinate frame that the message data is associated with.

            * The fields are all of float32 type, 4 bytes long each.
            * All of the point cloud data is written to a single data row (height of message is 1)
            '''
            header = rospy.Header()
            header.stamp = rospy.Time(nsecs=timestamp)
            header.frame_id = 'custom_frame_imu'
            pc2 = point_cloud2.create_cloud(header=header, fields=PC_FIELDS, points=point_cloud)
            bag.write('/custom_lidar_points', pc2, t=header.stamp)
        
          

        # insert IMU data
        for row_i, imu_row in imu_data.iterrows():
            header = rospy.Header()
            header.stamp = rospy.Time(nsecs=row_i)
            header.frame_id = 'custom_frame_imu'
            
            imu_msg = Imu(header=header,
                        #   orientation=imu_row[7:11],
                          angular_velocity=Vector3(imu_row['gyro_x'],imu_row['gyro_y'],imu_row['gyro_z']),
                          linear_acceleration=Vector3(imu_row['acc_x'],imu_row['acc_y'],imu_row['acc_z']),
                        #   orientation_covariance=imu_row[29:38],
                        #   angular_velocity_covariance=imu_row[20:29],
                        #   linear_acceleration_covariance=imu_row[11:20],
                        )

            bag.write("/imu", imu_msg, header.stamp)

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i","--input_dir",type=Path, help="absolute path of input dir")
    parser.add_argument("-o","--output_path",type=Path, help="absolute path of output dir")

    # parser.add_argument("-d","--device_type",type="str",input default="phone")
    myvars = parser.parse_args()
    phone2bag(myvars.input_dir, myvars.output_path)