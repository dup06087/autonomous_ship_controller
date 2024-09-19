import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import open3d.core as o3c
import os
import numpy as np
import math
import time
from utils import log_message
import json

class ICPHandler:
    def __init__(self, gnss_handler, experiment_folder):
        # self.gnss_handler = gnss_handler
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp.txt")
        self.prev_scan = None
        self.prev_heading_change = 0
        self.prev_x_change = 0
        self.log_list = []
        self.prev_time_icp_consuming = None
        
        d = 0.2  # 로봇이 x축으로 이동한 예상 거리 (단위: meters)
        self.icp_initial_guess = np.array([[1, 0, 0, d],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        
        # GNSS 데이터를 사용한 초기화
        self.initialize_position()

        # 이전 값을 초기화된 GNSS 값으로 설정
        # self.prev_x = self.current_x
        # self.prev_y = self.current_y
        # self.prev_heading = self.current_heading

        self.log_file = open(self.icp_data_file, "w")
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)
        
        

    def initialize_position(self):
        """유효한 GNSS 값을 받을 때까지 대기하며 초기화"""
        while True:
            """ original
            latitude = self.gnss_handler.current_value['latitude']
            longitude = self.gnss_handler.current_value['longitude']
            heading = self.gnss_handler.current_value['heading']
            """

            latitude = 37.62888010
            longitude = 127.0784448
            heading = 334.436
            
            if latitude is not None and longitude is not None and heading is not None:
                self.prev_latitude = latitude
                self.prev_longitude = longitude
                self.prev_heading = heading
                log_message(f"Initial position set: lat={self.prev_latitude}, lon={self.prev_longitude}, heading={self.prev_heading}")
                break
            else:
                log_message("Waiting for valid GNSS data to initialize position...")
                time.sleep(1)  # 1초 대기 후 다시 시도

    def run(self):
        rospy.spin()

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
    
    def icp_callback(self, status):
        try:
            current_time = time.time()
            time_consumed = current_time - self.prev_time_icp_consuming
            # Extracting information from the status dictionary
            iteration_index = status['iteration_index'].item()
            fitness = status['fitness'].cpu().item()
            inlier_rmse = status['inlier_rmse'].cpu().item()
            # transformation = status['transformation'].cpu().numpy()

            # Create a log message
            log_data = {
                'iteration_index': iteration_index,
                'fitness': fitness,
                'inlier_rmse': inlier_rmse,
                'time_consumed' : time_consumed
                # 'transformation': transformation.tolist()  # Convert numpy array to list for easier logging
            }
            
            self.log_list.append(log_data)
            
            print("log data : ", log_data)
            # print(f"Iteration: {iteration_index}, Fitness: {fitness}, Inlier RMSE: {inlier_rmse}, Time Consumed: {time_consumed}")
            # print(f"Transformation:\n{transformation}")
            self.prev_time_icp_consuming = current_time
            
        except Exception as e:
            print("icp callback error : ", e)
            
            
    def save_current_log_callback(self):
        try:         
            log_file = os.path.join(self.experiment_folder, "icp_log.txt")
            with open(log_file, 'a') as f:
                f.write(f"{self.log_list}\n")

            self.log_list = []
        except Exception as e:
            print("icp callback error : ", e)
            
    def lidar_callback(self, data):
        prev_time = time.time()
        # time_diff = rospy.Time.now() - data.header.stamp
        ros_time_now = rospy.get_rostime()
        time_diff = ros_time_now - data.header.stamp    
        if time_diff.to_sec() > 0.1:
            print("f : ", time_diff.to_sec())
            return

        try:
            cloud = self.point_cloud2_to_o3d(data)

            min_bound = np.array([-10, -10, -0.5])
            max_bound = np.array([10, 10, 2])
            cloud = self.crop_roi(cloud, min_bound, max_bound)
            ship_body_bounds = {'min': [-2, -2, -0.6], 'max': [2, 2, 0.31]}
            cloud = self.remove_ship_body(cloud, ship_body_bounds)
            cloud = self.downsample(cloud)

            if self.prev_scan is not None:
                
                theta = np.radians(self.prev_heading_change) 
                self.icp_initial_guess = np.array([[np.cos(theta), -np.sin(theta), 0, self.prev_x_change],
                    [np.sin(theta), np.cos(theta),  0, 0],
                    [0,             0,              1, 0],
                    [0,             0,              0, 1]])
                
                # reg_p2p = o3d.pipelines.registration.registration_icp(
                #     cloud.to_legacy(), self.prev_scan.to_legacy(), 0.5,
                #     self.icp_initial_guess,
                #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
                # transf = reg_p2p.transformation
                
                self.prev_time_icp_consuming = time.time()
                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud, 
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(
                        relative_fitness=1e-6,
                        relative_rmse=1e-6,
                        max_iteration=10
                    ),
                    callback_after_iteration=self.icp_callback
                )
                self.save_current_log_callback()
                
                transf = reg_gicp.transformation.cpu().numpy()  # Convert the result back to a numpy array


                if reg_gicp.fitness > 0.2:
                    # prev_time = time.time()

                    current_x = 0
                    current_y = 0
                    # current_z = 0
                    
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    heading_change = np.degrees(rotation_euler[2])
                    heading_change = math.trunc(heading_change * 10) / 10
                    # print("heading change check positive or negative : ", heading_change)
                    current_heading = self.prev_heading - heading_change
                    if current_heading > 180:
                        current_heading -= 360
                        
                    current_heading = current_heading % 360  # Normalize heading to [0, 360)

                    # print("돌리면서 0 > 360 > 0 >260 및 반대 방향도 확인 : current_heading : ", current_heading)

                    floored_lat = self.floor_to_eight_decimal_places(translation[0])
                    floored_lon = self.floor_to_eight_decimal_places(translation[1])

                    current_x += floored_lat
                    current_y += floored_lon
                    print("current_x : ", current_x, "current_y : ", current_y)
                    
                    lat, lon = self.calculate_new_position(
                        self.prev_latitude,
                        self.prev_longitude,
                        current_x,
                        current_y,
                        self.prev_heading
                    )
                    
                    self.log_file.write(f"{rospy.Time.now().to_sec()}, {lat}, {lon}, {current_heading}\n")

                    # 현재 값을 이전 값으로 갱신
                    
                    self.prev_latitude = lat
                    self.prev_longitude = lon
                    self.prev_heading = current_heading
                    self.prev_heading_change = heading_change
                    self.prev_x_change = current_x
                    self.prev_y_change = current_y
                    # print("done")
                else:
                    log_message("ICP fitness low")
                    
            self.prev_scan = cloud
        except Exception as e:
            log_message(f"ICP error: {e}")

        log_message(f"ICP time: {time.time() - prev_time}")

    def floor_to_eight_decimal_places(self, value):
        return math.trunc(value * 10**2) / 10**2

    def crop_roi(self, cloud, min_bound, max_bound):
        min_bound_tensor = o3c.Tensor(min_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        max_bound_tensor = o3c.Tensor(max_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        bbox = o3d.t.geometry.AxisAlignedBoundingBox(min_bound_tensor, max_bound_tensor)
        cropped_cloud = cloud.crop(bbox)
        return cropped_cloud

    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading):
        # Convert degrees to radians
        # if heading > 180:
        #     heading -= 360
            
        # heading_rad = math.radians(-heading)
        # delta_north = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        # delta_east = -delta_x * math.sin(heading_rad) - delta_y * math.cos(heading_rad)
        
        heading_rad = math.radians(heading)
        delta_north = delta_x * math.cos(heading_rad) - -delta_y * math.sin(heading_rad)
        delta_east = delta_x * math.sin(heading_rad) + -delta_y * math.cos(heading_rad)
        
        # Earth radius in meters
        R = 6378137.0
            # WGS84 ellipsoid constants
        a = 6378137.0  # Equatorial radius in meters
        e = 0.08181919  # Eccentricity
        
        lat_rad = math.radians(lat)
        R_m = a * (1 - e**2) / (1 - e**2 * math.sin(lat_rad)**2)**1.5
    
        delta_lat = delta_north / R_m
        
        # Calculate new latitude
        # delta_lat = delta_north / R
        new_lat = lat + math.degrees(delta_lat)

        # Calculate new longitude
        delta_lon = delta_east / (R * math.cos(math.radians(lat)))
        new_lon = lon + math.degrees(delta_lon)

        return new_lat, new_lon


    # def calculate_new_position(self, lat, lon, delta_x, delta_y, heading):
    #     # 변환된 heading: 90도에서 theta를 빼서 동쪽을 기준으로 변환
    #     heading = heading + 90
    #     if heading > 180:
    #         heading -= 360
    #     elif heading < 180:
    #         heading += 360
            
    #     heading_rad = math.radians(heading)  # 90 - theta로 설정

    #     # 회전 행렬을 사용한 변환
    #     delta_north = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
    #     delta_east = delta_x * math.sin(heading_rad) + delta_y * math.cos(heading_rad)

    #     print("delta_north:", delta_north, "delta_east:", delta_east)

    #     # 위도와 경도를 새로운 delta 값으로 업데이트
    #     R = 6378137.0  # 지구 반지름 (미터 단위)
    #     delta_lat = delta_north / R
    #     new_lat = lat + math.degrees(delta_lat)
    #     delta_lon = delta_east / (R * math.cos(math.radians(lat)))
    #     new_lon = lon + math.degrees(delta_lon)

    #     return new_lat, new_lon


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

    def save_data(self):
        self.log_file.close()

if __name__ == "__main__":
    rospy.init_node("icp_node")
    # gnss_handler = GNSSHandler()  # Assumes this class is defined elsewhere
    experiment_folder = "/path/to/experiment_folder"  # Set the correct path
    icp_handler = ICPHandler(gnss_handler, experiment_folder)
    icp_handler.run()
