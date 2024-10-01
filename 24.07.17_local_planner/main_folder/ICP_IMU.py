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

class IMUCorrector:
    def __init__(self, main_instance):
        print("IMUCorrector Initialized")
        self.boat = main_instance
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=2)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # IMU deltas (relative to last ICP result)
        self.dheading = 0  # Heading delta
        self.dheading_step = 0
        
        # Velocity initialized to zero        
        self.prev_time = None

    def imu_callback(self, data):
        # print('imu : ', rospy.Time.now())

        """Accumulate IMU changes for the next ICP step."""
        current_time = rospy.Time.now()
        time_diff = (current_time - data.header.stamp).to_sec()
        if time_diff > 0.1:
            return
        
        if self.prev_time is None:
            self.prev_time = current_time
            print("Initializing prev_time:", self.prev_time)
            return  # Skip the first calculation since we can't compute delta_time yet
        
        if self.boat.cnt_gnss_signal_error == 0:
            self.dheading = 0
            self.prev_time = current_time
            return
        
        # If prev_time is None (i.e., first callback), initialize it


        # Calculate delta time between IMU callbacks
        delta_time = (current_time - self.prev_time).to_sec()

        # Extract angular velocity for heading (z-axis rotation, i.e., yaw rate)
        angular_velocity_z = data.angular_velocity.z  # Angular velocity around z-axis (yaw rate)

        # Update heading using angular velocity (change in heading = angular velocity * time)
        self.dheading_step = angular_velocity_z * delta_time * 180 / math.pi
        self.dheading += self.dheading_step 

        self.prev_time = current_time

    def pre_icp_reset(self):
        """Reset only the IMU deltas before ICP starts, keep velocities intact."""
        self.dheading = 0
        # print("IMU deltas reset before ICP, velocities intact.")

    def run(self):
        rospy.spin()  # 이 스레드는 imu_callback을 계속해서 구독하고 처리
        
class ICPHandler:
    def __init__(self, main_instance, imu_corrector):
        self.imu_corrector = imu_corrector
        self.main_instance = main_instance
        # self.experiment_folder = experiment_folder
        # self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        self.prev_scan = None

        # ICP global position (latitude, longitude, heading)
        self.prev_latitude = None
        self.prev_longitude = None
        self.prev_heading = None
        
        # ICP initial guess
        self.icp_initial_guess = np.eye(4)
        # self.icp_result_pub = rospy.Publisher('/icp_result', Float64MultiArray, queue_size=1)
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        # self.log_file = open(self.icp_data_file, "a")
        self.processed_time = None

        self.dnorth = 0
        self.deast = 0
        self.dheading = 0

        self.prev_x_moved = 0
        self.prev_y_moved = 0
        self.prev_heading_changed = 0
        self.icp_value_ready = False
        
        self.current_value = {"latitude" : None, "longitude" : None, "heading" : None, "COG" : None, "velocity" : None, "forward_velociy" : None, "rotational_velocity" : None}
        
        rospy.Timer(rospy.Duration(0.2), self.update_main_instance)


    def update_main_instance(self, event):
        """5Hz로 main_instance의 current_value 값을 업데이트"""
        if self.main_instance.current_value is not None:
            latitude = self.main_instance.current_value.get('latitude')
            longitude = self.main_instance.current_value.get('longitude')
            heading = self.main_instance.current_value.get('heading')

            if latitude is not None and longitude is not None and heading is not None:
                self.prev_latitude = latitude
                self.prev_longitude = longitude
                self.prev_heading = heading
                # print(f"Main instance updated: lat={latitude}, lon={longitude}, heading={heading}")
            else:
                pass
                # print("Main instance values are None, skipping update.")
        else:
            print("Main instance current_value is None, skipping update.")


    def lidar_callback(self, data):
        # print('lidar : ', rospy.Time.now())
        try:
            # Get the current time and check how much time has passed since the last scan
            current_time = rospy.Time.now()
            time_diff = (current_time - data.header.stamp).to_sec()
            
            if time_diff > 0.1:
                return
            if self.processed_time == None:
                self.processed_time = current_time
                return

            # Convert PointCloud2 message to Open3D format
            cloud = self.point_cloud2_to_o3d(data)

            cloud = self.downsample(cloud)

            cloud = self.translate_pointcloud(cloud, distance=0.5)  # 0.5m를 예시로 적용

            if self.prev_scan is not None and self.main_instance.flag_icp_execute:
                self.dheading = self.imu_corrector.dheading
                self.imu_corrector.pre_icp_reset()
                self.apply_imu_to_icp_guess()

                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud,
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=10)
                )

                # If ICP fitness is good, update position and heading
                if reg_gicp.fitness > 0.8:
                    transf = reg_gicp.transformation.cpu().numpy()
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    # Update heading based on ICP result
                    icp_heading_change = np.degrees(rotation_euler[2])
                    current_heading = round((self.prev_heading - icp_heading_change) % 360, 3)

                    # Calculate new global position based on ICP translation result
                    lat, lon, v_x, v_y = self.calculate_new_position(
                        self.prev_latitude, self.prev_longitude,
                        -translation[2], translation[0], self.prev_heading, (data.header.stamp - self.processed_time).to_sec()
                    )
                    lat = round(lat, 8)
                    lon = round(lon, 8)
                    # Update global position with ICP result
                    self.prev_latitude = lat
                    self.prev_longitude = lon
                    self.prev_heading = current_heading
                    # self.imu_corrector.post_icp_reset(v_x, v_y)
                    self.prev_x_moved = translation[0]
                    self.prev_y_moved = translation[1]
                    self.prev_heading_changed = icp_heading_change
                    
                    # Publish updated ICP result
                    # self.publish_icp_result(lat, lon, current_heading)

                    # Log the result to file
                    # self.log_icp_result_to_file(lat, lon, current_heading, data.header.stamp)
                    
                    rotational_velocity = round((icp_heading_change)/(data.header.stamp - self.processed_time).to_sec() , 3)# rad/s
                    velocity = round(math.sqrt(v_x**2+v_y**2), 3)
                    
                    self.icp_value_ready= True
                    self.current_value = {"latitude" : lat, "longitude" : lon, "heading" : current_heading, "forward_velocity" : v_x, "velocity" : velocity, "rotational_velcity" : rotational_velocity}
                    self.processed_time = current_time
                    print("ICP done")
                    
            else:
                self.icp_value_ready= False
                self.current_value = {"latitude" : None, "longitude" : None, "heading" : None, "COG" : None, "velocity" : None, "forward_velociy" : None, "rotational_velocity" : None}

                print("prev_scan none : ", rospy.Time.now())
            # Store the current scan for the next iteration
            self.prev_scan = cloud

        except Exception as e:
            print(f"ICP error: {e}")

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
        """Convert local ICP dx, dy to latitude and longitude updates, and calculate velocities."""
        if heading > 180:
            heading -=360
        heading_rad = math.radians(heading-90)
        
        # Convert local dx, dy to global north/east deltas
        delta_north = delta_x * math.cos(heading_rad) - (delta_y) * math.sin(heading_rad)
        delta_east = delta_x * math.sin(heading_rad) + (delta_y) * math.cos(heading_rad)

        # Earth radius in meters
        R = 6378137.0
        lat_rad = math.radians(lat)

        # Convert delta_north and delta_east to latitude and longitude changes
        delta_lat = delta_north / R
        delta_lon = delta_east / (R * math.cos(lat_rad))

        # Update latitude and longitude
        new_lat = lat + math.degrees(delta_lat)
        new_lon = lon + math.degrees(delta_lon)

        # Calculate velocities based on the position deltas and time difference
        v_x = delta_x / delta_time
        v_y = delta_y / delta_time

        print(f"New Lat: {new_lat}, New Lon: {new_lon}, v_x: {v_x}, v_y: {v_y}, delta_time: {delta_time}")
        return new_lat, new_lon, v_x, v_y
    
    def apply_imu_to_icp_guess(self):
        """Use the IMU-based deltas to set the initial guess for ICP."""
        # dx dy는 dnorth deast기준이므로 이전 위치 heading으로 해야함. 현재X 
        # heading_rad = np.radians(self.prev_heading) 
    
        # # Compute local frame deltas using the inverse rotation matrix
        # dx = self.dnorth * math.cos(heading_rad) + self.dnorth * math.sin(heading_rad)
        # dy = self.dnorth * math.sin(heading_rad) - self.dnorth * math.cos(heading_rad)

        #이거는 dheading()
        heading_diff = np.radians(self.dheading)
        # heading_diff = np.radians(self.prev_heading_changed)
        # print("dheading : ", self.dheading)
        # print("dx dy : ", dx, dy)
        print("prev x y : ", self.prev_x_moved, self.prev_y_moved)
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, self.prev_x_moved],
            [np.sin(heading_diff), np.cos(heading_diff),  0, self.prev_y_moved],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])
        # print("self.icp_initial_guess : ", self.icp_initial_guess)

    def publish_icp_result(self, lat, lon, heading):
        msg = Float64MultiArray()
        msg.data = [lat, lon, heading]
        self.icp_result_pub.publish(msg)

    def log_icp_result_to_file(self, lat, lon, heading, stamp):
        """Logs the ICP result to a file in JSON format with time, dx, dy, dheading."""
        # Convert ROS time (stamp) to the required format (HHMMSS.milis)
        sec = stamp.secs  # Seconds since epoch
        nsec = stamp.nsecs  # Nanoseconds part

        # Convert seconds to datetime
        rostime = datetime.datetime.fromtimestamp(sec) - datetime.timedelta(hours=9)
        # Format the time as HHMMSS.milis
        formatted_time = rostime.strftime("%H%M%S") + f".{int(nsec / 1e6):03d}"

        # Prepare the log entry with additional dx, dy, dheading
        log_entry = {
            'lat': lat,
            'lon': lon,
            'heading': heading,
            'time': formatted_time,  # Add the time key in the requested format
            'dnorth': self.dnorth,  # IMU-corrected dx (north)
            'deast': self.deast,   # IMU-corrected dy (east)
            'dheading': self.dheading,  # IMU-corrected heading delta
            'vx' : self.imu_corrector.vx,
            'vy' : self.imu_corrector.vy
        }

        # Write to log file in JSON format
        with open(self.icp_data_file, 'a') as f:
            f.write(json.dumps(log_entry) + '\n')
        print(f"Logged ICP result: {log_entry}")


    def point_cloud2_to_o3d(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(np.array(points), dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return cloud

    def downsample(self, cloud, voxel_size=0.1):
        return cloud.voxel_down_sample(voxel_size)
    
    def translate_pointcloud(self, cloud, distance):
        """
        PointCloud를 주어진 거리만큼 차량의 heading 방향으로 평행 이동하는 함수.
        
        :param cloud: Open3D PointCloud 객체
        :param distance: 이동시킬 거리 (미터 단위, GNSS와 LiDAR 간의 거리)
        :param heading: 차량의 heading 값 (라디안 단위)
        :return: 평행 이동된 PointCloud
        """
        # heading 각도에 맞춰 이동할 x, y 성분 계산        
        # PointCloud의 모든 점들을 dx, dy 만큼 이동시킴
        

        cloud = cloud.translate((0, distance, 0))

        return cloud

    def run(self):
        rospy.spin()  # 이 스레드는 imu_callback을 계속해서 구독하고 처리
        
if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")

    # 이후의 메인 연산 시작
    print("Starting main computation...")
    # Initialize IMUCorrector and ICPHandler
    imu_corrector = IMUCorrector()
    experiment_folder = "./ekf_results"  # Set correct path for saving results
    icp_handler = ICPHandler(imu_corrector, experiment_folder)

    # Run both IMU and ICP
    rospy.spin()
