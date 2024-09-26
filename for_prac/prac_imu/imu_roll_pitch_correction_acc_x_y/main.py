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
# initial_latitude_raw = 3737.7326613
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7064656
# initial_longitude_direction = 'E'
# initial_heading = 8.955

'''rhombus'''
initial_latitude_raw = 3737.7328295
initial_latitude_direction = 'N'
initial_longitude_raw = 12704.7068961
initial_longitude_direction = 'E'
initial_heading = 335.797

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
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)

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
    
        # Correct linear acceleration (in local frame)
        linear_acceleration = data.linear_acceleration
        print("raw data : ", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z)
        corrected_acc = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch)

        # Update velocity in local frame: v = v0 + a * t
        self.vx += corrected_acc[0] * delta_time
        self.vy += corrected_acc[1] * delta_time

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

    def rotate_acceleration(self, ax, ay, az, roll, pitch):
        """Rotate acceleration data from IMU's local frame to global frame without yaw rotation."""
        
        # Roll 회전 행렬
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        # Pitch 회전 행렬
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        # Yaw를 고려하지 않고 Roll과 Pitch만 적용
        R = np.dot(R_y, R_x)
        
        # 가속도 벡터
        acceleration = np.array([ax, ay, az])
        
        # 변환된 가속도
        corrected_acceleration = np.dot(R, acceleration)
        print("corrected acc : ", corrected_acceleration)    
        return corrected_acceleration
    
if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")
    warm_up_open3d_cuda()

    # 이후의 메인 연산 시작
    print("Starting main computation...")
    # Initialize IMUCorrector and ICPHandler
    imu_corrector = IMUCorrector()
    experiment_folder = "./ekf_results"  # Set correct path for saving results

    # Run both IMU and ICP
    rospy.spin()
