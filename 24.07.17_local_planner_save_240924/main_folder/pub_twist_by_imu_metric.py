import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, Quaternion
import math
import os
from datetime import datetime
import numpy as np

class VelocityPublisher:
    def __init__(self, mother_instance):
        print("VelocityPublisher Publishing")
        # self.mother_instance = mother_instance
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.previous_time = rospy.Time.now()

        # 초기 위도, 경도, heading 설정 (예시값 사용)
        self.initial_latitude =  37.62918851  # 초기 위도
        self.initial_longitude = 127.07945248  # 초기 경도
        self.initial_heading = 0  # 초기 heading, 단위는 라디안

        # 현재 위치 및 heading 초기화
        self.current_latitude = self.initial_latitude
        self.current_longitude = self.initial_longitude
        self.current_heading = self.initial_heading

        # 속도 초기화
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.angular_velocity = 0.0

        # 지구 반지름 (미터 단위)
        self.earth_radius = 6378137.0

        # 로그 파일 생성
        self.log_file = self.create_log_file()
        self.sum_x = 0
        self.sum_y = 0
        
    def create_log_file(self):
        # 현재 날짜와 시간을 기반으로 폴더 및 파일 이름 생성
        now = datetime.now()
        directory_name = now.strftime("./imu_log/%Y%m%d_%H%M%S")
        os.makedirs(directory_name, exist_ok=True)
        file_path = os.path.join(directory_name, "log.txt")
        return open(file_path, 'w')

    def quaternion_to_euler(self, x, y, z, w):
        # 쿼터니언을 오일러 각도로 변환하는 함수
        # Roll (x축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def rotate_acceleration(self, ax, ay, az, roll, pitch, yaw):
        # 회전 행렬 계산
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
        
        # 전체 회전 행렬
        R = np.dot(R_z, np.dot(R_y, R_x))
        
        # 측정된 가속도 벡터
        acceleration = np.array([ax, ay, az])
        
        # 보정된 가속도 계산
        corrected_acceleration = np.dot(R, acceleration)
        
        return corrected_acceleration

    def rotate_angular_velocity(self, wx, wy, wz, roll, pitch, yaw):
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
        
        angular_velocity = np.array([wx, wy, wz])
        corrected_angular_velocity = np.dot(R, angular_velocity)
        
        return corrected_angular_velocity
    
    def imu_callback(self, data):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()

        ##### RPY 구하기 (Quaternion to Euler)
        orientation = data.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        ##### 선형 속도 구하기
        linear_acceleration = data.linear_acceleration
        corrected_acc = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch, yaw)
        
        # 선형 속도 계산 (이전 속도 + 보정된 가속도 * 시간)
        linear_acc_x_trunc = corrected_acc[0]
        linear_acc_y_trunc = corrected_acc[1]
        self.linear_velocity_x += linear_acc_x_trunc * delta_time
        self.linear_velocity_y += linear_acc_y_trunc * delta_time

        # 이동 거리 계산 (미터 단위)
        delta_x = self.linear_velocity_x * delta_time  # X축 방향 이동 거리
        delta_y = self.linear_velocity_y * delta_time  # Y축 방향 이동 거리
        
        ##### 각속도 데이터
        angular_velocity = data.angular_velocity
        corrected_angular_velocity = self.rotate_angular_velocity(
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            roll, pitch, yaw
        )

        # Here we use the corrected angular velocity for heading calculation
        angular_velocity_z_rad = corrected_angular_velocity[2]  # Z-axis (yaw) component in radians
        angular_velocity_z = math.degrees(angular_velocity_z_rad)
        
        self.current_heading -= angular_velocity_z * delta_time
        self.current_heading = self.current_heading % 360
        
        ##### Position 업데이트 (x, y) in metric units
        heading_rad = math.radians(self.current_heading)
        
        delta_x_corrected = delta_x * math.cos(heading_rad) - -delta_y * math.sin(heading_rad)
        delta_y_corrected = delta_x * math.sin(heading_rad) + -delta_y * math.cos(heading_rad)
        
        # x, y positions are updated using the corrected deltas
        self.sum_x += delta_x_corrected
        self.sum_y += delta_y_corrected

        # Print debug information
        print("linear_acc_x_trunc:", linear_acc_x_trunc)
        print("linear_acc_y_trunc:", linear_acc_y_trunc)
        print("linear_velocity_x:", self.linear_velocity_x)
        print("linear_velocity_y:", self.linear_velocity_y)
        print("delta_x, delta_y:", delta_x, delta_y)
        print("Corrected delta_x, delta_y:", delta_x_corrected, delta_y_corrected)
        print("Cumulative Position -> X:", self.sum_x, " Y:", self.sum_y)
        
        # Save the cumulative position to a log file
        self.log_file.write(f"{self.sum_x},{self.sum_y}\n")
        
        # Update the previous time for the next iteration
        self.previous_time = current_time


    def heading_to_quaternion(self, heading):
        # heading (yaw) 값을 quaternion으로 변환
        return Quaternion(0, 0, math.sin(heading / 2.0), math.cos(heading / 2.0))

    def __del__(self):
        # 로그 파일 닫기
        self.log_file.close()

if __name__ == '__main__':
    try:
        rospy.init_node('velocity_publisher', anonymous=True)
        velocity_publisher = VelocityPublisher(None)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
