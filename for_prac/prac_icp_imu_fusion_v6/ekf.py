import rospy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import os
import json
from scipy.spatial.transform import Rotation as R

class LiDARIMUEKF:
    def __init__(self):
        # rospy.init_node('lidar_imu_ekf', anonymous=True)

        # 지구 반지름 (미터 단위)
        self.R = 6378137.0
        
        # 상태 벡터: [경도, 위도, theta, v_east, v_north]
        self.xEst = np.zeros((5, 1))  # 초기 상태
        self.xEst[0, 0] = 127.078415438  # 초기 경도 설정 (예시 값)
        self.xEst[1, 0] = 37.628837589   # 초기 위도 설정 (예시 값)
        self.xEst[2, 0] = 8.133 # 초기 theta 설정 (예시 값)
        
        self.PEst = np.eye(5)*1  # 공분산 행렬 초기화
        # 자이로스코프 노이즈 (rad^2/s^2) at 25 Hz
        # 자이로스코프 노이즈 (rad^2/s^2) at 10 Hz
        gyro_noise_variance = 0.001

        latlon_noise_vairance = (1e-4)**2 # 4, 2
        
        # 가속도계 노이즈 (m^2/s^4) at 10 Hz
        acc_noise_variance = 0.01
        
        # 5x5 공분산 행렬 (가속도 3축, 자이로스코프 2축)
        self.Q = np.diag([latlon_noise_vairance, latlon_noise_vairance, gyro_noise_variance, 
                        acc_noise_variance, acc_noise_variance])
        # self.Q = np.eye(5)
        self.R_cov = np.eye(5) * (1e-8)**2  # 측정 잡음 공분산 행렬 (LiDAR 측정 오차)
        
        self.previous_time = None

        # 결과 저장을 위한 파일
        self.log_file_path = os.path.join(os.getcwd(), "ekf_result.txt")
        with open(self.log_file_path, "w") as f:
            f.write("")  # 파일 초기화
            
        # Publisher for the final result
        self.ekf_pub = rospy.Publisher('/ekf_output', Float64MultiArray, queue_size=10)

        # IMU와 LiDAR 콜백 분리
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/icp_result', Float64MultiArray, self.lidar_callback)

        print('ready')
        
    def imu_callback(self, imu_data):
        # 현재 시간 확인 및 delta_time 계산
        if self.previous_time is None:
            self.previous_time = rospy.Time.now()
            return
        
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        # IMU에서 받은 데이터 처리 (가속도 및 각속도)
        ax, ay, az, wz = self.process_imu_data(imu_data)
        
        # 쿼터니언 -> RPY 변환
        orientation = imu_data.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # 가속도 보정 (free acceleration 추출)
        free_acc_x, free_acc_y, free_acc_z = self.remove_gravity_and_correct(ax, ay, az, roll, pitch, yaw)
        # IMU 데이터로 Predict 단계 수행
        self.predict(free_acc_x, free_acc_y, wz, delta_time)

        # 상태 퍼블리시 (예측된 상태)
        self.publish_state()

    def lidar_callback(self, msg):
        # LiDAR에서 받은 ICP 결과 (위도, 경도, 헤딩)를 처리합니다.
        lidar_lat = msg.data[0]  # 위도
        lidar_lon = msg.data[1]  # 경도
        lidar_theta = msg.data[2]  # 헤딩
        lidar_vx = msg.data[3]  # veast
        lidar_vy = msg.data[4]  # vnorth
        # LiDAR 데이터로 Update 단계 수행
        self.update(lidar_lon, lidar_lat , lidar_theta, lidar_vx, lidar_vy)

        # 상태 퍼블리시
        self.publish_state()

    def save_to_file(self, latitude, longitude, heading, veast, vnorth, aeast=None, anorth=None, delta_time=None, ax= None, ay=None):
        """ 위도, 경도, 헤딩을 JSON 형식으로 텍스트 파일에 저장 """
        data = {
            'lat': latitude,
            'lon': longitude,
            'heading': heading,
            'veast': veast,
            'vnorth': vnorth,
            'aeast': aeast,
            'anorth': anorth,
            'delta_time': delta_time,
            'ax': ax,
            'ay': ay
        }
        with open(self.log_file_path, "a") as f:
            f.write(json.dumps(data) + "\n")

    def remove_gravity_and_correct(self, ax, ay, az, roll, pitch, yaw):
        # Step 1: 원래 회전 행렬 생성
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        
        # Step 2: Yaw를 0으로 만드는 보정 (Yaw correction)
        yaw_correction = R.from_euler('z', -yaw)
        corrected_rotation = yaw_correction * rotation
        
        # Step 3: 전역 좌표계에서의 중력 벡터 (z축 기준 9.81m/s²)
        gravity_global = np.array([0, 0, 9.81])
        
        # Step 4: 보정된 로컬 좌표계로 중력 벡터를 변환
        gravity_local = corrected_rotation.inv().apply(gravity_global)
        
        # Step 5: 가속도에서 중력 성분 제거
        free_acc_x = ax - gravity_local[0]
        free_acc_y = ay - gravity_local[1]
        free_acc_z = az - gravity_local[2]

        return free_acc_x, free_acc_y, free_acc_z

    
    def quaternion_to_euler(self, x, y, z, w):
        rotation = R.from_quat([x, y, z, w])
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)  # 라디안 단위로 변환
        return roll, pitch, yaw
    
    def process_imu_data(self, imu_data):
        """IMU 데이터를 처리하여 가속도 및 각속도 추출."""
        ax = imu_data.linear_acceleration.x
        ay = imu_data.linear_acceleration.y
        az = imu_data.linear_acceleration.z
        wz = imu_data.angular_velocity.z
        return ax, ay, az, wz

    # def process_lidar_data(self, lidar_data):
    #     """LiDAR 데이터를 처리하여 위치와 헤딩 추출 (임시 값)"""
    #     lidar_x = 0.0  # 실제 LiDAR 데이터를 처리하는 부분이 필요합니다.
    #     lidar_y = 0.0
    #     lidar_theta = 0.0
    #     return lidar_x, lidar_y, lidar_theta

    def predict(self, ax, ay, wz, delta_time):

        """IMU 데이터를 이용한 예측 단계 수행."""
        theta = self.xEst[2, 0]  # 현재 헤딩 값
        theta_save = theta
        print("theta : ", theta)

        if theta > 180:
            theta -=360
            
        theta = math.radians(theta)
        # Convert local dx, dy to global north/east deltas
        a_north = ax * math.cos(theta) - (-ay) * math.sin(theta)
        a_east = ax * math.sin(theta) + (-ay) * math.cos(theta)
        
        # 로컬 좌표계에서 전역 좌표계로 가속도 변환 (회전 행렬 적용)
        # a_north = ax * np.cos(theta) - -ay * np.sin(theta)
        # a_east = ax * np.sin(theta) + -ay * np.cos(theta)
        # print("a east, a north : ", a_east, a_north)

        # 속도 예측
        self.xEst[3, 0] += a_east * delta_time  # 동쪽 속도 예측
        self.xEst[4, 0] += a_north * delta_time  # 북쪽 속도 예측
        print("(IMU) ax vx ay vy: ", a_east, self.xEst[3, 0], a_north, self.xEst[4, 0], delta_time)

        # 동서 및 남북 이동 거리를 계산
        delta_x = self.xEst[3, 0] * delta_time + 0.5 * a_east * delta_time**2  # 동쪽 이동 거리 (미터)
        delta_y = self.xEst[4, 0] * delta_time + 0.5 * a_north * delta_time**2  # 북쪽 이동 거리 (미터)
        # print("delta_x delta_y : ", delta_x, delta_y)

        # 동서 및 남북 이동 거리를 위도, 경도로 변환
        delta_lat = (delta_y / self.R) * (180.0 / np.pi)  # 북쪽 이동 거리 -> 위도 변화
        delta_lon = (delta_x / (self.R * np.cos(np.radians(self.xEst[1, 0])))) * (180.0 / np.pi)  # 동쪽 이동 거리 -> 경도 변화
        # print("delta lat, delta lon : ", delta_lat, delta_lon)
        # 상태 벡터의 위도, 경도 업데이트
        self.xEst[1, 0] += delta_lat  # 위도 업데이트
        self.xEst[0, 0] += delta_lon  # 경도 업데이트

        # 방향 예측
        self.xEst[2, 0] -= math.degrees(wz) * delta_time  # 방향 예측

        # print("(EKF)current vx vy : ", self.xEst[3, 0], self.xEst[4, 0], ", dx dy : ", delta_x, delta_y)
        
        
        # 상태 전이 모델 자코비안 행렬(F) 계산
        # F = np.eye(5)

        # # 경도와 위도에 대한 속도의 영향을 반영한 자코비안
        # F[0, 3] = delta_time / (self.R * np.cos(np.radians(self.xEst[1, 0])))  # 경도에 대한 동쪽 속도 영향
        # F[1, 4] = delta_time / self.R  # 위도에 대한 북쪽 속도 영향

        # # theta에 대한 경도와 위도 편미분 (속도 변환 영향 반영)
        # F[0, 2] = (-self.xEst[3, 0] * delta_time * np.sin(theta) - self.xEst[4, 0] * delta_time * np.cos(theta)) / (self.R * np.cos(np.radians(self.xEst[1, 0])))  # 경도에 대한 theta 영향
        # F[1, 2] = (self.xEst[3, 0] * delta_time * np.cos(theta) - self.xEst[4, 0] * delta_time * np.sin(theta)) / self.R  # 위도에 대한 theta 영향

        theta = math.radians(theta_save)

        # 상태 전이 모델 자코비안 행렬(F) 계산 (Matlab 결과 반영)
        F = np.eye(5)

        # 자코비안 행렬 업데이트
        F[0, 1] = (np.sin(np.radians(self.xEst[1, 0])) * (((-ay * np.cos(theta)) / 2 + (ax * np.sin(theta)) / 2) * delta_time**2 + self.xEst[3, 0] * delta_time)) / (self.R * np.cos(np.radians(self.xEst[1, 0]))**2)
        F[0, 2] = (delta_time**2 * ((ax * np.cos(theta)) / 2 - (-ay * np.sin(theta)) / 2)) / (self.R * np.cos(np.radians(self.xEst[1, 0])))
        F[0, 3] = delta_time / (self.R * np.cos(np.radians(self.xEst[1, 0])))
        F[1, 2] = -(delta_time**2 * (-ay * np.cos(theta) + ax * np.sin(theta))) / (2 * self.R)
        F[1, 4] = delta_time / self.R
        F[3, 2] = delta_time * (ax * np.cos(theta) - -ay * np.sin(theta))
        F[4, 2] = -delta_time * (-ay * np.cos(theta) + ax * np.sin(theta))
        
        # 공분산 행렬 예측
        self.PEst = F @ self.PEst @ F.T + self.Q
        self.save_to_file(self.xEst[1,0], self.xEst[0,0], self.xEst[2,0], self.xEst[3,0], self.xEst[4,0], a_east, a_north, delta_time, ax, ay)
        # print("IMU callback : ", self.xEst[1,0], self.xEst[0,0], self.xEst[2,0])
        print("(IMU) : ", self.xEst[2,0])

    def update(self, lidar_x, lidar_y, lidar_theta, lidar_vx, lidar_vy):
        # print("update callback")
        """LiDAR 데이터를 이용한 업데이트 단계 수행 (속도 포함)."""
        
        # LiDAR에서 얻은 위치, 속도 정보를 EKF의 상태 벡터에 반영합니다.
        z = np.array([[lidar_x],    # 경도
                    [lidar_y],    # 위도
                    [lidar_theta], # 헤딩
                    [lidar_vx],   # 동쪽 속도 (v_east)
                    [lidar_vy]])  # 북쪽 속도 (v_north)

        # 측정 모델 자코비안 행렬 H (위치와 방향뿐만 아니라 속도도 측정)
        H = np.array([[1, 0, 0, 0, 0],  # 경도에 대한 측정
                    [0, 1, 0, 0, 0],  # 위도에 대한 측정
                    [0, 0, 1, 0, 0],  # 헤딩에 대한 측정
                    [0, 0, 0, 1, 0],  # 동쪽 속도에 대한 측정
                    [0, 0, 0, 0, 1]]) # 북쪽 속도에 대한 측정

        # 잔차 계산 (y = z - Hx)
        y = z - H @ self.xEst

        # 칼만 이득 계산
        S = H @ self.PEst @ H.T + self.R_cov  # 측정 공분산 행렬에 맞게 수정
        K = self.PEst @ H.T @ np.linalg.inv(S)

        # 상태 업데이트 (위치, 방향, 속도 업데이트)
        self.xEst = self.xEst + K @ y
        print("(ICP) : ", self.xEst[2,0])
        self.PEst = (np.eye(5) - K @ H) @ self.PEst

        # 상태 퍼블리시
        # self.publish_state()
        self.save_to_file(self.xEst[1,0], self.xEst[0,0], self.xEst[2,0], self.xEst[3,0], self.xEst[4,0])

        
    def publish_state(self):
        """상태 벡터 퍼블리시."""
        msg = Float64MultiArray()
        msg.data = [self.xEst[1, 0], self.xEst[0, 0], self.xEst[2, 0]]  # 위도, 경도, theta (헤딩)
        self.ekf_pub.publish(msg)

if __name__ == '__main__':
    try:
        ekf_node = LiDARIMUEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
