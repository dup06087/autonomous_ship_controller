import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import message_filters
from geometry_msgs.msg import Vector3Stamped  # angular velocity는 Vector3Stamped 타입

class IMUEKF:
    def __init__(self):
        rospy.init_node('imu_ekf', anonymous=True)
        
        # Publisher for corrected EKF output
        self.correction_pub = rospy.Publisher('/imu/ekf_corrected', Float64MultiArray, queue_size=10)
        
        # Subscribe to free acceleration and angular velocity using message_filters for sync
        free_acc_sub = message_filters.Subscriber('/filter/free_acceleration', Vector3Stamped)
        ang_vel_sub = message_filters.Subscriber('/imu/angular_velocity', Vector3Stamped)

        # Time synchronizer to sync both topics
        ts = message_filters.ApproximateTimeSynchronizer([free_acc_sub, ang_vel_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.imu_callback)

        self.previous_time = rospy.Time.now()

        # 상태 벡터: [x, y, yaw]
        self.xEst = np.zeros((3, 1))  # [x, y, yaw]
        self.PEst = np.eye(3)  # 공분산 행렬
        self.Q = np.eye(3) * 0.001 # 시스템 잡음 공분산 행렬
        # 자이로스코프 (yaw 각속도)의 RMS 노이즈 및 공분산
        gyro_noise_density = 0.003  # °/s/√Hz
        gyro_noise_r = (gyro_noise_density * np.sqrt(25)) ** 2  # RMS 노이즈의 제곱

        # 가속도계의 RMS 노이즈 및 공분산 (x, y 축의 가속도)
        accel_noise_density = 70e-6 * 9.81  # m/s²/√Hz로 변환
        accel_noise_r = (accel_noise_density * np.sqrt(25)) ** 2  # RMS 노이즈의 제곱

        # 측정 잡음 공분산 행렬 R (가속도와 각속도)
        self.R = np.diag([accel_noise_r*10000, accel_noise_r*10000, gyro_noise_r*10000])

        # self.R = np.diag([10, 10, 10])  # 측정 잡음 공분산 행렬

        # 속도 변수 (가속도를 적분하여 속도를 추정)
        self.v = np.zeros((2, 1))  # [v_x, v_y]

    def motion_model(self, x, u, dt, v):
        # 상태 전이 모델: 회전 행렬 적용
        yaw = x[2, 0]  # 현재 yaw 값
        wz = u[2, 0]  # 입력 벡터에서 각속도 사용

        # 회전 행렬을 사용해 가속도를 전역 좌표계로 변환
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        # 가속도를 전역 좌표계로 변환
        acc_x_global = cos_yaw * u[0, 0] - sin_yaw * -u[1, 0]
        acc_y_global = sin_yaw * u[0, 0] + cos_yaw * -u[1, 0]

        # 속도를 업데이트 (가속도를 적분하여 속도를 계산)
        v[0, 0] += acc_x_global * dt  # x 방향 속도 업데이트
        v[1, 0] += acc_y_global * dt  # y 방향 속도 업데이트

        # 새로운 상태 값 예측 (속도를 통해 위치 및 yaw 업데이트)
        x_pred = np.zeros_like(x)
        x_pred[0, 0] = x[0, 0] + v[0, 0] * dt  # x 위치 예측
        x_pred[1, 0] = x[1, 0] + v[1, 0] * dt  # y 위치 예측
        x_pred[2, 0] = x[2, 0] - wz * dt  # yaw 업데이트 (각속도 이용)

        return x_pred, v

    def jacobian_F(self, x, u, dt):
        # 자코비안 행렬 (상태 전이 모델)
        yaw = x[2, 0]  # 현재 yaw 값

        # 자코비안 행렬 계산
        jF = np.array([
            [1.0, 0.0, -u[0, 0] * dt * np.sin(yaw)],  # dx/dyaw
            [0.0, 1.0, u[1, 0] * dt * np.cos(yaw)],   # dy/dyaw
            [0.0, 0.0, 1.0]                          # dyaw/dyaw
        ])
        return jF

    def imu_callback(self, free_acc_data, ang_vel_data):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()

        # 선형 가속도 데이터 (소수점 3째 자리에서 절삭)
        ax = math.trunc(free_acc_data.vector.x * 1000) / 1000
        ay = math.trunc(free_acc_data.vector.y * 1000) / 1000

        # 각속도 데이터 (소수점 3째 자리에서 절삭)
        wz = math.trunc(ang_vel_data.vector.z * 1000) / 1000  # 각속도 (yaw 변화량)

        # 입력 벡터 업데이트: [acc_x, acc_y, wz]
        u = np.array([[ax], [ay], [wz]])

        # EKF 예측 단계: motion model을 통해 상태 예측
        xPred, self.v = self.motion_model(self.xEst, u, delta_time, self.v)
        jF = self.jacobian_F(self.xEst, u, delta_time)
        PPred = jF @ self.PEst @ jF.T + self.Q  # 공분산 예측

        # 자코비안 H 계산 (관측 모델)
        jH = np.eye(3)  # 간단히 단위 행렬로 설정

        # 측정 업데이트 단계 (측정값: acc_x, acc_y, wz)
        z = np.array([[ax], [ay], [wz]])  # 측정 데이터 (free acceleration과 각속도 이용)
        y = z - jH @ xPred  # 잔차 계산
        S = jH @ PPred @ jH.T + self.R  # 측정 공분산
        K = PPred @ jH.T @ np.linalg.inv(S)  # 칼만 이득 계산

        # 상태 업데이트: 보정
        self.xEst = xPred + K @ y
        self.PEst = (np.eye(3) - K @ jH) @ PPred  # 공분산 업데이트

        # 보정된 데이터 퍼블리시
        corrected_data = Float64MultiArray()
        corrected_data.data = [self.xEst[0, 0], self.xEst[1, 0], self.xEst[2, 0]*180/math.pi]  # [x, y, yaw]
        self.correction_pub.publish(corrected_data)

        self.previous_time = current_time
        print("callback")

if __name__ == '__main__':
    try:
        imu_ekf = IMUEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
