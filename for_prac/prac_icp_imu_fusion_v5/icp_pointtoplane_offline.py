import rospy
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import open3d.core as o3c
import os
import numpy as np
import math
import time
import json
import datetime
from scipy.spatial.transform import Rotation as R
import sys
import signal
import rosbag

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 초기 GPS 값 설정
initial_latitude_raw = 3737.7326613
initial_latitude_direction = 'N'
initial_longitude_raw = 12704.7064656
initial_longitude_direction = 'E'
initial_heading = 8.955

def convert_to_degrees(value):
    degrees = int(value / 100)  # 도
    minutes = value - (degrees * 100)  # 분
    return degrees + (minutes / 60)

def convert_lat_lon(lat_value, lat_direction, lon_value, lon_direction):
    latitude = convert_to_degrees(lat_value)
    longitude = convert_to_degrees(lon_value)
    
    if lat_direction == 'S':
        latitude = -latitude  # 남반구일 경우 음수로 변환
    if lon_direction == 'W':
        longitude = -longitude  # 서경일 경우 음수로 변환
    
    return latitude, longitude

# 위도, 경도 도 단위로 변환
initial_latitude, initial_longitude = convert_lat_lon(
    initial_latitude_raw, initial_latitude_direction, 
    initial_longitude_raw, initial_longitude_direction
)

# CUDA 장치 설정
device = o3c.Device("CUDA:0")  # CUDA 장치 0을 사용

def warm_up_open3d_cuda():
    print("Warming up CUDA using Open3D-Core...")

    tensor_a = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)
    tensor_b = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)

    tensor_c = tensor_a + tensor_b

    result = tensor_c.cpu().numpy()
    print("CUDA warm-up complete.")
    return result

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        self.dnorth = 0
        self.deast = 0
        self.dheading = 0
        self.dheading_step = 0
        self.vx = 0
        self.vy = 0
        self.prev_heading = initial_heading
        self.prev_time = None

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        time_diff = (current_time - data.header.stamp).to_sec()
        if time_diff > 0.1:
            return
        if self.prev_time is None:
            self.prev_time = current_time
            print("Initializing prev_time:", self.prev_time)
            return
        
        delta_time = (current_time - self.prev_time).to_sec()
        angular_velocity_z = data.angular_velocity.z
        self.dheading_step = angular_velocity_z * delta_time * 180 / math.pi
        self.dheading += self.dheading_step
        current_heading = (self.prev_heading - self.dheading_step) % 360
        heading_rad = math.radians(current_heading)

        orientation = data.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        linear_acceleration = data.linear_acceleration
        free_acc = self.remove_gravity_and_correct(
            linear_acceleration.x, 
            linear_acceleration.y, 
            linear_acceleration.z, 
            roll, pitch, yaw
        )
        
        self.vx += free_acc[0] * delta_time
        self.vy += free_acc[1] * delta_time
        global_v_north = self.vx * math.cos(heading_rad) - -self.vy * math.sin(heading_rad)
        global_v_east = self.vx * math.sin(heading_rad) + -self.vy * math.cos(heading_rad)

        self.dnorth += global_v_north * delta_time
        self.deast += global_v_east * delta_time

        self.prev_heading = current_heading
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

    def pre_icp_reset(self):
        """Reset only the IMU deltas before ICP starts, keep velocities intact."""
        self.dnorth = 0
        self.deast = 0
        self.dheading = 0
    
    def post_icp_reset(self, vx, vy):
        """Reset both deltas and velocities after ICP has completed."""
        self.vx = vx
        self.vy = vy
        
class ICPHandler:
    def __init__(self, imu_corrector, experiment_folder):
        self.imu_corrector = imu_corrector
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        
        with open(self.icp_data_file, 'w') as f:
            f.write('')
        self.prev_scan = None
        self.prev_latitude = initial_latitude
        self.prev_longitude = initial_longitude
        self.prev_heading = initial_heading
        self.icp_initial_guess = np.eye(4)
        self.processed_time = None
        self.prev_x_moved = 0
        self.prev_y_moved = 0

    def lidar_callback(self, data, current_time):
        try:
            time_diff = (current_time - data.header.stamp).to_sec()
            if time_diff > 0.1:
                return
            if self.processed_time is None:
                self.processed_time = current_time
                return

            self.dnorth = self.imu_corrector.dnorth
            self.deast = self.imu_corrector.deast
            self.dheading = self.imu_corrector.dheading

            self.imu_corrector.pre_icp_reset()
            
            cloud = self.point_cloud2_to_o3d(data)
            cloud = self.downsample(cloud)
            cloud = self.estimate_normals(cloud)

            if self.prev_scan is not None:
                self.prev_scan = self.estimate_normals(self.prev_scan)
                self.apply_imu_to_icp_guess()

                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud,
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
                )

                if reg_gicp.fitness > 0.2:
                    transf = reg_gicp.transformation.cpu().numpy()
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    icp_heading_change = np.degrees(rotation_euler[2])
                    current_heading = (self.prev_heading - icp_heading_change) % 360

                    lat, lon, v_x, v_y = self.calculate_new_position(
                        self.prev_latitude, self.prev_longitude,
                        translation[0], -translation[1], self.prev_heading, 0.1
                    )

                    self.prev_x_moved = translation[0]
                    self.prev_y_moved = -translation[1]
                    
                    self.prev_latitude = lat
                    self.prev_longitude = lon
                    self.prev_heading = current_heading
                    self.imu_corrector.post_icp_reset(v_x, v_y)

                    self.log_icp_result_to_file(lat, lon, current_heading, current_time, translation)

            self.prev_scan = cloud

        except Exception as e:
            print(f"ICP error: {e}")

    def estimate_normals(self, cloud, radius=5.0):
        legacy_cloud = cloud.to_legacy()
        legacy_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
        )
        tensor_cloud = o3d.t.geometry.PointCloud.from_legacy(legacy_cloud, device=device)
        return tensor_cloud

    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading, delta_time):
        heading_rad = math.radians(heading)
        delta_north = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        delta_east = delta_x * math.sin(heading_rad) + delta_y * math.cos(heading_rad)
        R = 6378137.0
        lat_rad = math.radians(lat)
        delta_lat = delta_north / R
        delta_lon = delta_east / (R * math.cos(lat_rad))
        new_lat = lat + math.degrees(delta_lat)
        new_lon = lon + math.degrees(delta_lon)
        v_x = delta_x / delta_time
        v_y = delta_y / delta_time
        return new_lat, new_lon, v_x, v_y
    
    def apply_imu_to_icp_guess(self):
        heading_rad = np.radians(self.prev_heading)
        dx = self.imu_corrector.dnorth * math.cos(heading_rad) + self.imu_corrector.deast * math.sin(heading_rad)
        dy = self.imu_corrector.dnorth * math.sin(heading_rad) - self.imu_corrector.deast * math.cos(heading_rad)
        heading_diff = np.radians(self.dheading)
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, self.prev_x_moved],
            [np.sin(heading_diff), np.cos(heading_diff),  0, self.prev_y_moved],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])

    def log_icp_result_to_file(self, lat, lon, heading, stamp, translation):
        sec = stamp.to_sec()
        formatted_time = datetime.datetime.fromtimestamp(sec).strftime("%H:%M:%S")
        log_entry = {
            "time": formatted_time,
            "lat": lat,
            "lon": lon,
            "heading": heading,
            "translation[0]": translation[0],
            "translation[1]": translation[1],
            "translation[2]": translation[2]
        }
        with open(self.icp_data_file, 'a') as f:
            f.write(json.dumps(log_entry) + "\n")
        print(f"Logged ICP result: {log_entry}")

    def point_cloud2_to_o3d(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(np.array(points), dtype=o3c.float32, device=o3c.Device("CUDA:0")))
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

def process_bag(bag_file, imu_corrector, icp_handler):
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['/imu/data', '/velodyne_points']):
        if topic == '/imu/data':
            imu_corrector.imu_callback(msg)
        elif topic == '/velodyne_points':
            icp_handler.lidar_callback(msg, t)
    bag.close()

if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")
    warm_up_open3d_cuda()
    experiment_folder = "./ekf_results"
    imu_corrector = IMUCorrector()
    icp_handler = ICPHandler(imu_corrector, experiment_folder)
    bag_file = '../../../../rosbag_for_fusion/rect1.bag'  # 실제 rosbag 파일 경로 설정
    process_bag(bag_file, imu_corrector, icp_handler)
    print("Finished processing bag file.")
