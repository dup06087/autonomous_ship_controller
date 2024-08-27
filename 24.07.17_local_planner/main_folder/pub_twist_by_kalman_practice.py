import rospy
from sensor_msgs.msg import Imu
from filterpy.kalman import KalmanFilter
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class ImuKalmanFilterNode:
    def __init__(self):
        rospy.init_node('imu_kalman_filter_node')

        # 칼만 필터 초기화
        self.kf = KalmanFilter(dim_x=8, dim_z=3)  # 상태 벡터 크기 8 (p_x, p_y, theta, v_x, v_y, omega_z, a_x, a_y), 관측 벡터 크기 3 (a_x, a_y, angular_velocity_z)

        # 상태 전이 행렬 (State Transition Matrix)
        dt = 0.1  # 시간 간격 (예: 10ms)
        self.kf.F = np.array([[1, 0, 0, dt, 0,  0,  0.5*dt**2, 0],
                              [0, 1, 0, 0,  dt, 0,  0, 0.5*dt**2],
                              [0, 0, 1, 0,  0,  dt, 0, 0],
                              [0, 0, 0, 1,  0,  0,  dt, 0],
                              [0, 0, 0, 0,  1,  0,  0, dt],
                              [0, 0, 0, 0,  0,  1,  0, 0],
                              [0, 0, 0, 0,  0,  0,  1, 0],
                              [0, 0, 0, 0,  0,  0,  0, 1]])

        # 관측 모델 행렬 (Measurement Matrix)
        self.kf.H = np.array([[0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1],
                              [0, 0, 0, 0, 0, 1, 0, 0]])

        # 초기 상태 추정 (Initial state estimate)
        self.kf.x = np.zeros(8)

        # 공분산 행렬 (Covariance Matrix)
        self.kf.P *= 1000.  # 초기 불확실성
        self.kf.R = np.eye(3) * 0.00005  # 측정 불확실성 (가속도 및 각속도에 대한 노이즈)
        self.kf.Q = np.eye(8) * 10  # 프로세스 노이즈

        # 로그 파일 초기화
        self.log_file = open("position_log.txt", "w")

        # IMU 데이터 구독자 설정
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

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
    
    def imu_callback(self, msg):
        ##### RPY 구하기 (Quaternion to Euler)
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        ##### 선형 속도 구하기
        linear_acceleration = msg.linear_acceleration
        accel_corrected = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch, yaw)
        
        # 관측 벡터 생성 (보정된 가속도 및 자이로 데이터)
        z = np.array([accel_corrected[0], 
                      accel_corrected[1], 
                      msg.angular_velocity.z])

        # 칼만 필터 예측
        self.kf.predict()

        # 칼만 필터 업데이트
        self.kf.update(z)

        # 필터링된 위치를 로그로 남김
        p_x, p_y, v_x, a_x = self.kf.x[0], self.kf.x[1], self.kf.x[3], self.kf.x[6]
        self.log_file.write(f"{p_x},{p_y}\n")

        # 필터링된 상태 출력
        rospy.loginfo("Kalman Filtered State: p_x={:.2f}, v_y={:.2f}, a_x={:.2f}".format(p_x, v_x, a_x))
        # rospy.loginfo(f"Raw data: a_x={msg.linear_acceleration.x}, a_y={msg.linear_acceleration.y}, angular_velocity_z={msg.angular_velocity.z}")
        # rospy.loginfo(f"rpy corrected data : a_x={accel_corrected[0]}, a_y={accel_corrected[1]}")
    def __del__(self):
        self.log_file.close()
        
if __name__ == '__main__':
    try:
        node = ImuKalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
