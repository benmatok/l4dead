import rospy
import rosbag
from sensor_msgs.msg import Imu, PointField
from sensor_msgs import point_cloud2
from pathlib import Path
import numpy as np

def load_scan(scan_path: Path) -> 'tuple[float, np.ndarray]':
    '''method gets path of scan file, loads data and returns numpy array with timestamp'''
    scan = np.loadtxt(scan_path)
    timestamp = float(scan_path.stem)
    return timestamp, scan

def save_data_to_rosbag(scan_paths: list, imu_path: Path, output_file: Path) -> None:
    with rosbag.Bag(output_file, 'w') as bag:

        # handling point clouds
        for scan_path in scan_paths:
            timestamp, point_cloud = load_scan(scan_path) 
            
            '''
            Pointcloud2:
            * The Header message contains two main fields:
            -  stamp: Time object representing the time when the message was created
            -  frame_id: string representing the coordinate frame that the message data is associated with.

            * The fields are all of float32 type, 4 bytes long each.
            * All of the point cloud data is written to a single data row (height of message is 1)
            '''

            # point cloud message, all fields are float32 type, 4 bytes long
            fields = [
                PointField(name='x',offset=0, datatype=PointField.FLOAT32, height=1),
                PointField(name='y',offset=4, datatype=PointField.FLOAT32, height=1),
                PointField(name='z',offset=8, datatype=PointField.FLOAT32, height=1),
                PointField(name='intensity',offset=12, datatype=PointField.FLOAT32, height=1),
                PointField(name='normal_x',offset=16, datatype=PointField.FLOAT32, height=1),
                PointField(name='normal_y',offset=20, datatype=PointField.FLOAT32, height=1),
                PointField(name='normal_z',offset=24, datatype=PointField.FLOAT32, height=1),
            ]
            
            header = rospy.Header()
            header.stamp = rospy.Time.from_sec(timestamp)
            header.frame_id = 'map'
            # TODO: check if point_cloud needs to be point_cloud.tolist()
            pc2 = point_cloud2.create_cloud(header=header, fields=fields, points=point_cloud)
            bag.write('/point_cloud_topic', pc2, t=header.stamp)

        # handling IMU data

        imu_data = np.loadtxt(imu_path)
        #TODO: figure out data order in columns
        for imu_row in imu_data:
            header = rospy.Header()
            header.stamp = rospy.Time.from_sec(imu_row[0])
            header.frame_id = 'map'
            
            imu_msg = Imu(header=header, orientation=imu_row[6:10],
            angular_velocity=imu_row[3:6],
            linear_acceleration=imu_row[:3],
            orientation_covariance=imu_row[28:37],
            angular_velocity_covariance=imu_row[19:28],
            linear_acceleration_covariance=imu_row[10:19],)

            bag.write("/imu", imu_msg, header.stamp)