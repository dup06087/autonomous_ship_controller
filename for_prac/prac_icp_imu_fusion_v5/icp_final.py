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

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 수동으로 입력한 원래 값 (도와 분 형식)


'''lane_direction'''
# initial_latitude_raw = 3737.7302554
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7049263
# initial_longitude_direction = 'E'
# initial_heading =8.133

'''lane_vertial_direction'''
# initial_latitude_raw = 3737.7337296
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7078467
# initial_longitude_direction = 'E'
# initial_heading =282.903


'''rect1'''
initial_latitude_raw = 3737.7326613
initial_latitude_direction = 'N'
initial_longitude_raw = 12704.7064656
initial_longitude_direction = 'E'
initial_heading = 8.955

'''rhombus'''
# initial_latitude_raw = 3737.7328295
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7068961
# initial_longitude_direction = 'E'
# initial_heading = 335.797

'''round1'''
# initial_latitude_raw = 3737.7327122
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7064321
# initial_longitude_direction = 'E'
# initial_heading =9.532



def convert_to_degrees(value):
    """
    위경도 값을 도 단위로 변환하는 함수.
    value: 도와 분으로 표시된 위경도 값 (예: 3737.7327122)
    반환: 도 단위로 변환된 값
    """
    degrees = int(value / 100)  # 도
    minutes = value - (degrees * 100)  # 분
    return degrees + (minutes / 60)

def convert_lat_lon(lat_value, lat_direction, lon_value, lon_direction):
    """
    위도와 경도 값을 도 단위로 변환하는 함수.
    lat_value: 위도 값 (예: 3737.7327122)
    lat_direction: 위도 방향 ('N' 또는 'S')
    lon_value: 경도 값 (예: 12704.7064321)
    lon_direction: 경도 방향 ('E' 또는 'W')
    반환: 도 단위로 변환된 위도와 경도
    """
    latitude = convert_to_degrees(lat_value)
    longitude = convert_to_degrees(lon_value)
    
    if lat_direction == 'S':
        latitude = -latitude  # 남반구일 경우 음수로 변환
    if lon_direction == 'W':
        longitude = -longitude  # 서경일 경우 음수로 변환
    
    return latitude, longitude



# 위도, 경도 도 단위로 자동 변환
initial_latitude, initial_longitude = convert_lat_lon(
    initial_latitude_raw, initial_latitude_direction, 
    initial_longitude_raw, initial_longitude_direction
)




# CUDA 장치 설정
device = o3c.Device("CUDA:0")  # CUDA 장치 0을 사용

def warm_up_open3d_cuda():
    print("Warming up CUDA using Open3D-Core...")

    # 간단한 텐서를 생성하고 CUDA 디바이스에 올림
    tensor_a = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)
    tensor_b = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)

    # 두 텐서를 더하는 간단한 연산을 수행하여 CUDA 연산을 실행
    tensor_c = tensor_a + tensor_b

    # 결과 확인을 위해 CPU로 다시 복사
    result = tensor_c.cpu().numpy()
    
    print("CUDA warm-up complete.")
    return result

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # IMU deltas (relative to last ICP result)
        self.dnorth = 0  # X position delta (local)
        self.deast = 0  # Y position delta (local)
        self.dheading = 0  # Heading delta
        self.dheading_step = 0
        
        # Velocity initialized to zero
        self.vx = 0  # Velocity in x-direction
        self.vy = 0  # Velocity in y-direction
        self.prev_heading = initial_heading
        
        self.prev_time = None

    def imu_callback(self, data):
        # print('imu : ', rospy.Time.now())

        """Accumulate IMU changes for the next ICP step."""
        current_time = rospy.Time.now()
        
        time_diff = (current_time - data.header.stamp).to_sec()

        if time_diff > 0.1:
            return
                    
        # If prev_time is None (i.e., first callback), initialize it
        if self.prev_time is None:
            self.prev_time = current_time
            print("Initializing prev_time:", self.prev_time)
            return  # Skip the first calculation since we can't compute delta_time yet

        # Calculate delta time between IMU callbacks
        delta_time = (current_time - self.prev_time).to_sec()

        # Extract angular velocity for heading (z-axis rotation, i.e., yaw rate)
        angular_velocity_z = data.angular_velocity.z  # Angular velocity around z-axis (yaw rate)

        # Update heading using angular velocity (change in heading = angular velocity * time)
        self.dheading_step = angular_velocity_z * delta_time * 180 / math.pi
        self.dheading += self.dheading_step 
        # print("dh_step, dh : ", self.dheading_step, self.dheading)
        # Update the current heading by adding the incremental heading change (dheading)
        # Use self.prev_heading for this; initialize it properly
        current_heading = (self.prev_heading - self.dheading_step) % 360  # Keep heading in [0, 360] range
        heading_rad = math.radians(current_heading)  # Convert to radians for calculations

        # Extract orientation and convert to roll, pitch, yaw
        orientation = data.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # 선형 가속도에서 중력 성분 제거 후 free acceleration 계산
        linear_acceleration = data.linear_acceleration
        free_acc = self.remove_gravity_and_correct(
            linear_acceleration.x, 
            linear_acceleration.y, 
            linear_acceleration.z, 
            roll, pitch, yaw
        )
        
        # Correct linear acceleration (in local frame)
        # print("linear_acc : ", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z)
        # corrected_acc = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch, yaw)

        # print("free_acc : ", free_acc)
        # print("corrected_acc : ", corrected_acc)
        
        # Update velocity in local frame: v = v0 + a * t
        self.vx += free_acc[0] * delta_time
        self.vy += free_acc[1] * delta_time
        print("velocity : ", self.vx, self.vy)
        # Transform velocity from local frame to global frame using current heading
        global_v_north = self.vx * math.cos(heading_rad) - -self.vy * math.sin(heading_rad)
        global_v_east = self.vx * math.sin(heading_rad) + -self.vy * math.cos(heading_rad)

        # Update deltas (dx = v * t, dy = v * t) in global frame
        self.dnorth += global_v_north * delta_time
        self.deast += global_v_east * delta_time
        # Print the updated IMU data
        # print(f"IMU vx: {self.vx}, vy: {self.vy}, dheading: {self.dheading}")
        # print(f"IMU global dnorth: {self.dnorth}, global deast: {self.deast}, heading: {current_heading}")

        # Update the previous time for the next callback
        self.prev_heading = current_heading
        self.prev_time = current_time

    def remove_gravity_and_correct(self, ax, ay, az, roll, pitch, yaw):
        # Yaw를 0으로 만들기 위한 회전 행렬 계산
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        yaw_correction = R.from_euler('z', -yaw)
        corrected_rotation = yaw_correction * rotation

        # 중력 벡터 계산 (z축 방향으로 9.81 m/s²)
        g = 9.81
        gravity = np.array([0, 0, g])

        # Yaw를 상쇄한 회전 행렬로 중력 벡터 변환
        rotation_matrix = corrected_rotation.as_matrix()
        gravity_local = rotation_matrix.T @ gravity

        # 가속도에서 중력 성분 제거 (free acceleration 계산)
        free_acc_x = ax - gravity_local[0]
        free_acc_y = ay - gravity_local[1]
        free_acc_z = az - gravity_local[2]

        return free_acc_x, free_acc_y, free_acc_z
    
    def pre_icp_reset(self):
        """Reset only the IMU deltas before ICP starts, keep velocities intact."""
        self.dnorth = 0
        self.deast = 0
        self.dheading = 0
        # print("IMU deltas reset before ICP, velocities intact.")
    
    def post_icp_reset(self, vx, vy):
        """Reset both deltas and velocities after ICP has completed."""
        self.vx = vx
        self.vy = vy
        # print("IMU deltas and velocities reset after ICP.")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
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

    def rotate_acceleration(self, ax, ay, az, roll, pitch, yaw):
        """Rotate acceleration data from IMU's local frame to global frame."""
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        R = np.dot(R_z, np.dot(R_y, R_x))
        acceleration = np.array([ax, ay, az])
        corrected_acceleration = np.dot(R, acceleration)
        
        return corrected_acceleration


class ICPHandler:
    def __init__(self, imu_corrector, experiment_folder):
        self.imu_corrector = imu_corrector
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        self.prev_scan = None

        # ICP global position (latitude, longitude, heading)
        self.prev_latitude = initial_latitude
        self.prev_longitude = initial_longitude
        self.prev_heading = initial_heading

        # ICP initial guess
        self.icp_initial_guess = np.eye(4)
        self.icp_result_pub = rospy.Publisher('/icp_result', Float64MultiArray, queue_size=1)
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        with open(self.icp_data_file, 'w') as f:
            f.write("")
            
        self.processed_time = None

        self.dnorth = 0
        self.deast = 0
        self.dheading = 0

        self.prev_x_moved = 0
        self.prev_y_moved = 0
        self.prev_heading_changed = 0
        
    def lidar_callback(self, data):
        # print('lidar : ', rospy.Time.now())
        try:
            # Get the current time and check how much time has passed since the last scan
            current_time = rospy.Time.now()
            time_diff = (current_time - data.header.stamp).to_sec()
            
            if time_diff > 0.1:
                # print("time diff : ", time_diff)
                return
            if self.processed_time == None:
                self.processed_time = current_time
                return

            self.dnorth = self.imu_corrector.dnorth
            self.deast = self.imu_corrector.deast
            self.dheading = self.imu_corrector.dheading

            # Reset IMU deltas at the start of the callback
            self.imu_corrector.pre_icp_reset()

            # Convert PointCloud2 message to Open3D format
            cloud = self.point_cloud2_to_o3d(data)

            cloud = self.downsample(cloud)

            # cloud = self.translate_pointcloud(cloud, distance=0.5)  # 0.5m를 예시로 적용


            if self.prev_scan is not None:
                # Use IMU's deltas as the initial guess for ICP
                self.apply_imu_to_icp_guess()

                # Perform ICP registration
                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud,
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
                )

                # If ICP fitness is good, update position and heading
                if reg_gicp.fitness > 0.8:
                    transf = reg_gicp.transformation.cpu().numpy()
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    # Update heading based on ICP result
                    icp_heading_change = np.degrees(rotation_euler[2])
                    current_heading = (self.prev_heading - icp_heading_change) % 360

                    self.calculate_matrix(transf)
                    
                    #Calculate new global position based on ICP translation result
                    lat, lon, v_x, v_y = self.calculate_new_position(
                        self.prev_latitude, self.prev_longitude,
                        translation[0], translation[1], self.prev_heading, (data.header.stamp - self.processed_time).to_sec()
                    )
                    
                    # Update global position with ICP result
                    self.prev_latitude = lat
                    self.prev_longitude = lon
                    self.prev_heading = current_heading
                    self.imu_corrector.post_icp_reset(v_x, v_y)
                    print("calculated velocity : ", v_x, v_y)
                    self.prev_x_moved = translation[0]
                    self.prev_y_moved = translation[1]
                    self.prev_heading_changed = icp_heading_change
                    
                    # Publish updated ICP result
                    self.publish_icp_result(lat, lon, current_heading)

                    # Log the result to file
                    self.log_icp_result_to_file(lat, lon, current_heading, data.header.stamp)
                    self.processed_time = current_time
            else:
                print("prev_scan none : ", rospy.Time.now())
            # Store the current scan for the next iteration
            self.prev_scan = cloud

        except Exception as e:
            print(f"ICP error: {e}")
        
    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading, delta_time):
        """Convert local ICP dx, dy to latitude and longitude updates, and calculate velocities."""
        if heading > 180:
            heading -=360
        heading_rad = math.radians(heading)
        
        # Convert local dx, dy to global north/east deltas
        delta_north = delta_x * math.cos(heading_rad) - (-delta_y) * math.sin(heading_rad)
        delta_east = delta_x * math.sin(heading_rad) + (-delta_y) * math.cos(heading_rad)

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
        heading_rad = np.radians(self.prev_heading) 
    
        # Compute local frame deltas using the inverse rotation matrix
        dx = self.dnorth * math.cos(heading_rad) + self.deast * math.sin(heading_rad)
        dy = self.dnorth * math.sin(heading_rad) - self.deast * math.cos(heading_rad)

        #이거는 dheading()
        heading_diff = np.radians(self.dheading)
        # heading_diff = np.radians(self.prev_heading_changed)
        # print("dheading : ", self.dheading)
        print("dx dy : ", dx, dy)
        # print("prev x y : ", self.prev_x_moved, self.prev_y_moved)
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, self.prev_x_moved],
            [np.sin(heading_diff), np.cos(heading_diff),  0, self.prev_y_moved],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])
        
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, dx],
            [np.sin(heading_diff), np.cos(heading_diff),  0, dy],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])
        
        # self.icp_initial_guess = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1,  0, 0],
        #     [0,             0,              1, 0],
        #     [0,             0,              0, 1]
        # ])
        
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

    def rotation_matrix_to_euler(self, rotation_matrix):
        """Convert a rotation matrix to Euler angles."""
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

if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")
    warm_up_open3d_cuda()

    # 이후의 메인 연산 시작
    print("Starting main computation...")
    # Initialize IMUCorrector and ICPHandler
    imu_corrector = IMUCorrector()
    experiment_folder = "./ekf_results"  # Set correct path for saving results
    icp_handler = ICPHandler(imu_corrector, experiment_folder)

    # Run both IMU and ICP
    rospy.spin()
