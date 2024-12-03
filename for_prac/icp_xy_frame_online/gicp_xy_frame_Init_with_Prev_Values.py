# Modified version of the code to use CPU instead of CUDA for GICP

import rospy
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import math
import time
import json
import datetime
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import sys
import signal
import os
import time

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        self.raw_imu_pub = rospy.Publisher('/example/imu', Imu, queue_size=10)  # Republish to /example/imu

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.dnorth = 0  
        self.deast = 0  
        self.dyaw = 0  
        self.dyaw_step = 0
        self.vx = 0  
        self.vy = 0  
        self.prev_heading = 0
        self.prev_time = None

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        time_diff = (current_time - data.header.stamp).to_sec()

        if time_diff > 0.1:
            return

        data.header.frame_id = "odom"  
        self.raw_imu_pub.publish(data)

        if self.prev_time is None:
            self.prev_time = current_time
            print("Initializing prev_time:", self.prev_time)
            return

        delta_time = (current_time - self.prev_time).to_sec()
        angular_velocity_z = data.angular_velocity.z  
        self.dyaw_step = angular_velocity_z * delta_time * 180 / math.pi
        self.dyaw += self.dyaw_step 
        self.prev_time = current_time

    def remove_gravity_and_correct(self, ax, ay, az, roll, pitch, yaw):
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        yaw_correction = R.from_euler('z', -yaw)
        corrected_rotation = yaw_correction * rotation
        g = 9.81
        gravity = np.array([0, 0, g])
        rotation_matrix = corrected_rotation.as_matrix()
        gravity_local = rotation_matrix.T @ gravity
        free_acc_x = ax - gravity_local[0]
        free_acc_y = ay - gravity_local[1]
        free_acc_z = az - gravity_local[2]

        return free_acc_x, free_acc_y, free_acc_z
    
    def pre_icp_reset(self):
        self.dyaw = 0

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

class ICPHandler:
    def __init__(self, imu_corrector, experiment_folder):
        self.imu_corrector = imu_corrector
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        self.prev_scan = None

        self.prev_x_global = 0
        self.prev_y_global = 0
        self.prev_yaw = 0
        self.prev_dheading = 0
        
        self.icp_initial_guess = np.eye(4)
        self.icp_result_pub = rospy.Publisher('/icp_result', Float64MultiArray, queue_size=1)
        self.odom_pub = rospy.Publisher('/example/odom', Odometry, queue_size=10)
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        with open(self.icp_data_file, 'w') as f:
            f.write("")

        self.processed_time = None

    def lidar_callback(self, data):
        
        # print(type(data))
        
        try:
            current_time = rospy.Time.now()
            time_diff = (current_time - data.header.stamp).to_sec()

            if time_diff > 0.1:
                return
            if self.processed_time is None:
                self.processed_time = current_time
                return

            self.dyaw = self.imu_corrector.dyaw
            self.imu_corrector.pre_icp_reset()
            cloud = self.point_cloud2_to_o3d(data)
            cloud = self.crop_roi(cloud, start=[-50, -50, -10], end=[50, 50, 10])

            # cloud = self.downsample(cloud)

            if self.prev_scan is not None:
                self.apply_imu_to_icp_guess()

                # Record the start time of ICP
                icp_start_time = time.time()

                # print(type(cloud))  # 예상: <class 'open3d.geometry.PointCloud'>
                # print(type(self.prev_scan))  # 예상: <class 'open3d.geometry.PointCloud'>
                
                # CPU-based GICP without CUDA
                reg_gicp = o3d.pipelines.registration.registration_generalized_icp(
                    cloud, self.prev_scan, 1.5, self.icp_initial_guess,
                    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-10, relative_rmse=1e-10, max_iteration=10)
                )

                # Record the end time of ICP
                icp_end_time = time.time()
                icp_duration = icp_end_time - icp_start_time

                if reg_gicp.fitness > 0.8:
                    transf = reg_gicp.transformation
                    translation = transf[:3, 3]
                    x_moved_local = translation[0]
                    y_moved_local = translation[1]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    icp_yaw_change = np.degrees(rotation_euler[2])
                    current_yaw = (self.prev_yaw + icp_yaw_change) % 360

                    time_diff = (data.header.stamp - self.processed_time).to_sec()
                    x_global, y_global, v_x_global, v_y_global, x_moved_global, y_moved_global = self.calculate_new_position(
                        self.prev_x_global, self.prev_y_global,
                        x_moved_local, y_moved_local, self.prev_yaw, time_diff
                    )

                    self.prev_x_global = x_global
                    self.prev_y_global = y_global
                    self.prev_yaw = current_yaw

                    self.publish_icp_result(x_global, y_global, current_yaw, translation[0], translation[1], v_x_global, v_y_global, icp_yaw_change)

                    # Log the ICP result along with the duration
                    self.log_icp_result_to_file(x_global, y_global, current_yaw, v_x_global, v_y_global, 
                                                x_moved_local, y_moved_local, x_moved_global, y_moved_global, 
                                                time_diff, data.header.stamp, icp_duration)

                    self.processed_time = current_time
                    self.prev_dheading = icp_yaw_change
                    print("icp done")
            self.prev_scan = cloud

        except Exception as e:
            print(f"ICP error: {e}")
    
    def crop_roi(self, pcd, start, end):
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=start, max_bound=end)
        cropped_pcd = pcd.crop(bbox)
        return cropped_pcd
    
    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading, delta_time):
        heading_rad = math.radians(heading)
        delta_x_global = delta_x * math.cos(heading_rad) - (delta_y) * math.sin(heading_rad)
        delta_y_global = delta_x * math.sin(heading_rad) + (delta_y) * math.cos(heading_rad)

        lat += delta_x_global
        lon += delta_y_global
        
        v_x_global = delta_x_global / delta_time
        v_y_global = delta_y_global / delta_time

        return lat, lon, v_x_global, v_y_global, delta_x_global, delta_y_global
    
    def apply_imu_to_icp_guess(self):
        heading_diff = np.radians(self.prev_dheading)
        # heading_diff = 0
        
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, 0],
            [np.sin(heading_diff), np.cos(heading_diff),  0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def publish_icp_result(self, lat, lon, heading, dx, dy, v_x, v_y, icp_heading_change):
        msg = Float64MultiArray()
        msg.data = [lat, lon, heading]
        self.icp_result_pub.publish(msg)
        self.publish_odometry(lat, lon, heading, v_x, v_y, icp_heading_change)

    def publish_odometry(self, lat, lon, heading, v_x, v_y, angular_velocity_z):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = lat
        odom_msg.pose.pose.position.y = lon
        odom_msg.pose.pose.position.z = 0

        heading_rad = math.radians(heading)
        rotation = R.from_euler('z', heading_rad)  
        quaternion = rotation.as_quat()  
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]  

        odom_msg.twist.twist.linear.x = v_x  
        odom_msg.twist.twist.linear.y = v_y  
        odom_msg.twist.twist.linear.z = 0

        odom_msg.twist.twist.angular.z = angular_velocity_z * math.pi / 180

        self.odom_pub.publish(odom_msg)
        
    def log_icp_result_to_file(self, lat, lon, heading, v_x, v_y, x_moved_local, y_moved_local, x_moved_global, y_moved_global, time_diff, stamp, icp_duration):
        """Logs the ICP result to a file in JSON format with time, dx, dy, dheading, and ICP duration."""
        # Convert ROS time (stamp) to the required format (HHMMSS.milis)
        sec = stamp.secs  
        nsec = stamp.nsecs  

        # Convert seconds to datetime
        rostime = datetime.datetime.fromtimestamp(sec) - datetime.timedelta(hours=9)
        # Format the time as HHMMSS.milis
        formatted_time = rostime.strftime("%H%M%S") + f".{int(nsec / 1e6):03d}"

        # Prepare the log entry with additional dx, dy, dheading, and ICP duration
        log_entry = {
            'x': lat,
            'y': lon,
            'yaw': heading,
            'v_x': v_x,
            'v_y': v_y,
            'x_moved_local': x_moved_local,
            'y_moved_local': y_moved_local,
            'x_moved_global': x_moved_global,
            'y_moved_global': y_moved_global,
            'time_diff': time_diff,
            'icp_duration': icp_duration,  # Add the ICP duration
            'time': formatted_time,  # Add the time key in the requested format
        }

        # Write to log file in JSON format
        with open(self.icp_data_file, 'a') as f:
            f.write(json.dumps(log_entry) + '\n')
        print(f"Logged ICP result: {log_entry}")

    def point_cloud2_to_o3d(self, cloud_msg):
        # 포인트 데이터에서 직접 배열로 변환
        cloud_array = np.array(list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(cloud_array)
        return cloud

    def downsample(self, cloud, voxel_size=0.1):
        return cloud.voxel_down_sample(voxel_size)

    def rotation_matrix_to_euler(self, rotation_matrix):
        R = np.array(rotation_matrix)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")

    # Initialize IMUCorrector and ICPHandler
    imu_corrector = IMUCorrector()
    experiment_folder = "./ekf_results"  
    icp_handler = ICPHandler(imu_corrector, experiment_folder)

    # Start the main computation loop
    rospy.spin()
