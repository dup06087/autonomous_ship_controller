import rospy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import os
import json

class LiDARIMUEKF:
    def __init__(self):
        rospy.init_node('lidar_imu_ekf', anonymous=True)

        # 지구 반지름 (미터 단위)
        self.R = 6378137.0
        
        # 상태 벡터: [경도, 위도, theta, v_east, v_north]
        self.xEst = np.zeros((5, 1))  # 초기 상태
        self.xEst[0, 0] = 127.07844109  # 초기 경도 설정 (예시 값)
        self.xEst[1, 0] = 37.62887769   # 초기 위도 설정 (예시 값)
        self.xEst[2, 0] = 8.955 # 초기 theta 설정 (예시 값)
        
        self.PEst = np.eye(5)  # 공분산 행렬 초기화
        # 자이로스코프 노이즈 (rad^2/s^2) at 25 Hz
        # 자이로스코프 노이즈 (rad^2/s^2) at 10 Hz
        gyro_noise_variance = 2.74e-4

        # 가속도계 노이즈 (m^2/s^4) at 10 Hz
        acc_noise_variance = 4.71e-4

        # 5x5 공분산 행렬 (가속도 3축, 자이로스코프 2축)
        self.Q = np.diag([acc_noise_variance, acc_noise_variance, acc_noise_variance, 
                        gyro_noise_variance, gyro_noise_variance])
        self.R_cov = np.eye(3) * 1e-6  # 측정 잡음 공분산 행렬 (LiDAR 측정 오차)
        
        self.previous_time = rospy.Time.now()

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
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time

        # IMU에서 받은 데이터 처리 (가속도 및 각속도)
        ax, ay, wz = self.process_imu_data(imu_data)

        # IMU 데이터로 Predict 단계 수행 (주기적으로)
        # self.predict(ax, ay, wz, delta_time)
        self.predict(ax, ay, wz, 0.1)
        # 상태 퍼블리시 (예측된 상태)
        self.publish_state()

    def lidar_callback(self, msg):
        # LiDAR에서 받은 ICP 결과 (위도, 경도, 헤딩)를 처리합니다.
        lidar_lat = msg.data[0]  # 위도
        lidar_lon = msg.data[1]  # 경도
        lidar_theta = msg.data[2]  # 헤딩

        # LiDAR 데이터로 Update 단계 수행
        self.update(lidar_lon, lidar_lat , lidar_theta)

        # 상태 퍼블리시
        self.publish_state()

    def save_to_file(self, latitude, longitude, heading):
        """ 위도, 경도, 헤딩을 JSON 형식으로 텍스트 파일에 저장 """
        data = {
            'lat': latitude,
            'lon': longitude,
            'heading': heading
        }
        with open(self.log_file_path, "a") as f:
            f.write(json.dumps(data) + "\n")

            
    def process_imu_data(self, imu_data):
        """IMU 데이터를 처리하여 가속도 및 각속도 추출."""
        ax = imu_data.linear_acceleration.x
        ay = imu_data.linear_acceleration.y
        wz = imu_data.angular_velocity.z
        return ax, ay, wz

    # def process_lidar_data(self, lidar_data):
    #     """LiDAR 데이터를 처리하여 위치와 헤딩 추출 (임시 값)"""
    #     lidar_x = 0.0  # 실제 LiDAR 데이터를 처리하는 부분이 필요합니다.
    #     lidar_y = 0.0
    #     lidar_theta = 0.0
    #     return lidar_x, lidar_y, lidar_theta

    def predict(self, ax, ay, wz, delta_time):

        """IMU 데이터를 이용한 예측 단계 수행."""
        theta = self.xEst[2, 0]  # 현재 헤딩 값
        print("theta : ", theta)
        theta = math.radians(theta)
        
        # 로컬 좌표계에서 전역 좌표계로 가속도 변환 (회전 행렬 적용)
        a_north = ax * np.cos(theta) - -ay * np.sin(theta)
        a_east = ax * np.sin(theta) + -ay * np.cos(theta)
        print("a east, a north : ", a_east, a_north)

        # 동서 및 남북 이동 거리를 계산
        delta_x = self.xEst[3, 0] * delta_time + 0.5 * a_east * delta_time**2  # 동쪽 이동 거리 (미터)
        delta_y = self.xEst[4, 0] * delta_time + 0.5 * a_north * delta_time**2  # 북쪽 이동 거리 (미터)
        print("delta_x delta_y : ", delta_x, delta_y)

        # 동서 및 남북 이동 거리를 위도, 경도로 변환
        delta_lat = (delta_y / self.R) * (180.0 / np.pi)  # 북쪽 이동 거리 -> 위도 변화
        delta_lon = (delta_x / (self.R * np.cos(np.radians(self.xEst[1, 0])))) * (180.0 / np.pi)  # 동쪽 이동 거리 -> 경도 변화
        print("delta lat, delta lon : ", delta_lat, delta_lon)
        # 상태 벡터의 위도, 경도 업데이트
        self.xEst[1, 0] += delta_lat  # 위도 업데이트
        self.xEst[0, 0] += delta_lon  # 경도 업데이트

        # 방향 예측
        self.xEst[2, 0] -= math.degrees(wz) * delta_time  # 방향 예측

        # 속도 예측
        self.xEst[3, 0] += a_east * delta_time  # 동쪽 속도 예측
        self.xEst[4, 0] += a_north * delta_time  # 북쪽 속도 예측

        # 상태 전이 모델 자코비안 행렬(F) 계산
        # F = np.eye(5)

        # # 경도와 위도에 대한 속도의 영향을 반영한 자코비안
        # F[0, 3] = delta_time / (self.R * np.cos(np.radians(self.xEst[1, 0])))  # 경도에 대한 동쪽 속도 영향
        # F[1, 4] = delta_time / self.R  # 위도에 대한 북쪽 속도 영향

        # # theta에 대한 경도와 위도 편미분 (속도 변환 영향 반영)
        # F[0, 2] = (-self.xEst[3, 0] * delta_time * np.sin(theta) - self.xEst[4, 0] * delta_time * np.cos(theta)) / (self.R * np.cos(np.radians(self.xEst[1, 0])))  # 경도에 대한 theta 영향
        # F[1, 2] = (self.xEst[3, 0] * delta_time * np.cos(theta) - self.xEst[4, 0] * delta_time * np.sin(theta)) / self.R  # 위도에 대한 theta 영향


        # 상태 전이 모델 자코비안 행렬(F) 계산 (Matlab 결과 반영)
        F = np.eye(5)

        # 자코비안 행렬 업데이트
        F[0, 1] = (np.sin(np.radians(self.xEst[1, 0])) * (((ay * np.cos(theta)) / 2 + (ax * np.sin(theta)) / 2) * delta_time**2 + self.xEst[3, 0] * delta_time)) / (self.R * np.cos(np.radians(self.xEst[1, 0]))**2)
        F[0, 2] = (delta_time**2 * ((ax * np.cos(theta)) / 2 - (ay * np.sin(theta)) / 2)) / (self.R * np.cos(np.radians(self.xEst[1, 0])))
        F[0, 3] = delta_time / (self.R * np.cos(np.radians(self.xEst[1, 0])))
        F[1, 2] = -(delta_time**2 * (ay * np.cos(theta) + ax * np.sin(theta))) / (2 * self.R)
        F[1, 4] = delta_time / self.R
        F[3, 2] = delta_time * (ax * np.cos(theta) - ay * np.sin(theta))
        F[4, 2] = -delta_time * (ay * np.cos(theta) + ax * np.sin(theta))
        
        # 공분산 행렬 예측
        self.PEst = F @ self.PEst @ F.T + self.Q
        self.save_to_file(self.xEst[1,0], self.xEst[0,0], self.xEst[2,0])
        print("IMU callback : ", self.xEst[1,0], self.xEst[0,0], self.xEst[2,0])

    def update(self, lidar_x, lidar_y, lidar_theta):
        print("update callback")
        """LiDAR 데이터를 이용한 업데이트 단계 수행."""
        z = np.array([[lidar_x], [lidar_y], [lidar_theta]])

        # 측정 모델 자코비안 행렬 H (LiDAR는 위치와 방향만 측정)
        H = np.array([[1, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0],
                      [0, 0, 1, 0, 0]])

        # 잔차 계산 (y = z - Hx)
        y = z - H @ self.xEst

        # 칼만 이득 계산
        S = H @ self.PEst @ H.T + self.R_cov
        K = self.PEst @ H.T @ np.linalg.inv(S)

        # 상태 업데이트
        self.xEst = self.xEst + K @ y
        self.PEst = (np.eye(5) - K @ H) @ self.PEst
        self.save_to_file(self.xEst[1,0], self.xEst[0,0], self.xEst[2,0])
        
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
