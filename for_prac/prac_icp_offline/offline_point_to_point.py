import rospy
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import open3d.core as o3c
import os
import numpy as np
import math
import time
import json
import datetime
import rosbag

import sys
import signal

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# 초기값 설정
initial_latitude_raw = 3737.7326613
initial_latitude_direction = 'N'
initial_longitude_raw = 12704.7064656
initial_longitude_direction = 'E'
initial_heading = 8.955

# 위경도 변환 함수
def convert_to_degrees(value):
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    return degrees + (minutes / 60)

def convert_lat_lon(lat_value, lat_direction, lon_value, lon_direction):
    latitude = convert_to_degrees(lat_value)
    longitude = convert_to_degrees(lon_value)
    
    if lat_direction == 'S':
        latitude = -latitude
    if lon_direction == 'W':
        longitude = -longitude
    
    return latitude, longitude

# 위도, 경도 도 단위로 변환
initial_latitude, initial_longitude = convert_lat_lon(
    initial_latitude_raw, initial_latitude_direction, 
    initial_longitude_raw, initial_longitude_direction
)

# CUDA 장치 설정
device = o3c.Device("CUDA:0")

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
        self.dheading = 0
        self.dheading_step = 0

        self.prev_heading = initial_heading
        self.prev_time = None

    def imu_callback(self, data, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            return

        delta_time = 0.1
        angular_velocity_z = data.angular_velocity.z
        self.dheading_step = angular_velocity_z * delta_time * 180 / math.pi
        
        self.dheading += self.dheading_step
        # print("imu callback : ", self.dheading)
        current_heading = (self.prev_heading - self.dheading_step) % 360

        self.prev_heading = current_heading
        self.prev_time = current_time

class ICPHandler:
    def __init__(self, imu_corrector, experiment_folder):
        self.imu_corrector = imu_corrector
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        self.prev_scan = None
        self.prev_latitude = initial_latitude
        self.prev_longitude = initial_longitude
        self.prev_heading = initial_heading
        self.icp_initial_guess = np.eye(4)
        self.processed_time = None
        self.log_file = open(self.icp_data_file, "a")
        self.prev_dx = 0
        self.prev_dy = 0

        with open(self.icp_data_file, 'w') as f:
            f.write("")
            

    def remove_ship_body(self, pcd, ship_body_bounds):
        # GPU에서 CPU로 데이터를 이동
        pcd_tensor = pcd.point.positions.to(o3c.Device("CPU:0"))

        # ship_body_bounds의 min과 max 값을 텐서로 변환
        min_bound = np.array(ship_body_bounds['min'])
        max_bound = np.array(ship_body_bounds['max'])

        # 범위에 포함되지 않는 포인트를 남김
        mask = ~((pcd_tensor[:, 0].numpy() >= min_bound[0]) &
                (pcd_tensor[:, 0].numpy() <= max_bound[0]) &
                (pcd_tensor[:, 1].numpy() >= min_bound[1]) &
                (pcd_tensor[:, 1].numpy() <= max_bound[1]) &
                (pcd_tensor[:, 2].numpy() >= min_bound[2]) &
                (pcd_tensor[:, 2].numpy() <= max_bound[2]))

        filtered_pcd_tensor = pcd_tensor.numpy()[mask]
        # 필터링된 포인트들을 GPU로 다시 전송
        filtered_pcd = o3d.t.geometry.PointCloud(o3c.Tensor(filtered_pcd_tensor, dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return filtered_pcd
    
    def lidar_callback(self, data, current_time):
        self.dheading = self.imu_corrector.dheading

        cloud = self.point_cloud2_to_o3d(data)
        # cloud = self.downsample(cloud)
        ship_body_bounds = {'min': [-2, -2, -5], 'max': [2, 2, 5]}
        cloud = self.remove_ship_body(cloud, ship_body_bounds)
        
        if self.prev_scan is not None:
            self.apply_imu_to_icp_guess()
            self.imu_corrector.dheading = 0


            # Perform ICP registration
            criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100,  # Increase the maximum iterations
                relative_fitness=1e-20,  # Set a very low fitness threshold
                relative_rmse=1e-20     # Set a very low RMSE threshold
            )
                
            reg_gicp = o3d.t.pipelines.registration.icp(
                source=cloud,
                target=self.prev_scan,
                max_correspondence_distance=1.5,
                init_source_to_target=self.icp_initial_guess,
                estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                # criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                criteria = criteria,
                callback_after_iteration=self.icp_callback

            )

            if reg_gicp.fitness > 0.8:
                transf = reg_gicp.transformation.cpu().numpy()
                translation = transf[:3, 3]
                # print("trans : ", translation)
                rotation_matrix = transf[:3, :3]
                rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)
                icp_heading_change = np.degrees(rotation_euler[2])
                current_heading = (self.prev_heading - icp_heading_change) % 360
                # dy = math.trunc(translation[1]*1e+3) *0.001
                # y value 살리기
                dy = math.trunc(-translation[1]*1e+3) *0.001
                dx = math.trunc(translation[0]*1e+3) *0.001
                lat, lon, v_x, v_y = self.calculate_new_position(
                    self.prev_latitude, self.prev_longitude,
                    dx, dy, self.prev_heading, 0.1
                )
                self.prev_dx = dx
                self.prev_dy = dy
                self.prev_latitude = lat
                self.prev_longitude = lon
                self.prev_heading = current_heading
                self.log_icp_result_to_file(lat, lon, current_heading, current_time, translation)
                self.processed_time = current_time
                
        self.prev_scan = cloud

        # ICP 콜백 함수 정의
    def icp_callback(self, status):
        try:
            # time_consumed = current_time - self.prev_time_icp_consuming
            # Extracting information from the status dictionary
            iteration_index = status['iteration_index'].item()
            fitness = status['fitness'].cpu().item()
            inlier_rmse = status['inlier_rmse'].cpu().item()
            transformation = status['transformation'].cpu().numpy()

            print(f"Iteration: {iteration_index}, Fitness: {fitness}, Inlier RMSE: {inlier_rmse}")
            print(f"Transformation:\n{transformation}")
            
            
        except Exception as e:
            print("icp callback error : ", e)
        
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
        heading_rad = np.radians(self.dheading)
        # dx = self.dnorth * math.cos(heading_rad)
        # dy = self.deast * math.sin(heading_rad)
        self.icp_initial_guess = np.array([
            [np.cos(heading_rad), -np.sin(heading_rad), 0, self.prev_dx],
            [np.sin(heading_rad), np.cos(heading_rad),  0, self.prev_dy],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])
        
        # self.icp_initial_guess = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1,  0, 0],
        #     [0,             0,              1, 0],
        #     [0,             0,              0, 1]
        # ])
                
        # print("prev dx dy, dheading", self.prev_dx, self.prev_dy, self.dheading)
        
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

def process_bag(bag_file, imu_corrector, icp_handler):
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['/imu/data', '/velodyne_points']):
        if topic == '/imu/data':
            imu_corrector.imu_callback(msg, t)
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
