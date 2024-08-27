import rospy
from sensor_msgs.msg import Imu
from filterpy.kalman import ExtendedKalmanFilter as EKF
import numpy as np
import math

class ImuEKFNode:
    def __init__(self):
        rospy.init_node('imu_ekf_node')

        # EKF 초기화
        self.ekf = EKF(dim_x=8, dim_z=3)  # 상태 벡터 크기 8 (p_x, p_y, theta, v_x, v_y, omega_z, a_x, a_y), 관측 벡터 크기 3 (a_x, a_y, angular_velocity_z)

        # 초기 상태 추정 (Initial state estimate)
        self.ekf.x = np.zeros(8)

        # 공분산 행렬 (Covariance Matrix)
        self.ekf.P *= 1000.  # 초기 불확실성
        self.ekf.R = np.eye(3) * 0.00005  # 측정 불확실성 (가속도 및 각속도에 대한 노이즈)
        self.ekf.Q = np.eye(8) * 10  # 프로세스 노이즈

        # 로그 파일 초기화
        self.log_file = open("position_log.txt", "w")

        # IMU 데이터 구독자 설정
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def rotate_acceleration(self, ax, ay, az, roll, pitch, yaw):
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

    def state_transition_function(self, x, dt):
        # 상태 전이 함수 (f(x))
        F = np.array([[1, 0, 0, dt, 0,  0,  0.5*dt**2, 0],
                      [0, 1, 0, 0,  dt, 0,  0, 0.5*dt**2],
                      [0, 0, 1, 0,  0,  dt, 0, 0],
                      [0, 0, 0, 1,  0,  0,  dt, 0],
                      [0, 0, 0, 0,  1,  0,  0, dt],
                      [0, 0, 0, 0,  0,  1,  0, 0],
                      [0, 0, 0, 0,  0,  0,  1, 0],
                      [0, 0, 0, 0,  0,  0,  0, 1]])
        return np.dot(F, x)

    def observation_function(self, x):
        # 관측 함수 (h(x))
        return np.array([x[6], x[7], x[5]])

    def jacobian_F(self, x, dt):
        # 상태 전이 함수의 야코비안 계산
        F_jac = np.array([[1, 0, 0, dt, 0,  0,  0.5*dt**2, 0],
                          [0, 1, 0, 0,  dt, 0,  0, 0.5*dt**2],
                          [0, 0, 1, 0,  0,  dt, 0, 0],
                          [0, 0, 0, 1,  0,  0,  dt, 0],
                          [0, 0, 0, 0,  1,  0,  0, dt],
                          [0, 0, 0, 0,  0,  1,  0, 0],
                          [0, 0, 0, 0,  0,  0,  1, 0],
                          [0, 0, 0, 0,  0,  0,  0, 1]])
        return F_jac

    def jacobian_H(self, x):
        # 관측 함수의 야코비안 계산
        H_jac = np.zeros((3, 8))
        H_jac[0, 6] = 1  # a_x에 대한 미분
        H_jac[1, 7] = 1  # a_y에 대한 미분
        H_jac[2, 5] = 1  # omega_z에 대한 미분
        return H_jac

    def imu_callback(self, msg):
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        linear_acceleration = msg.linear_acceleration
        accel_corrected = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch, yaw)
        
        z = np.array([accel_corrected[0], accel_corrected[1], msg.angular_velocity.z])

        # EKF 예측 단계
        dt = 0.1  # 시간 간격
        self.ekf.F = self.jacobian_F(self.ekf.x, dt)
        self.ekf.x = self.state_transition_function(self.ekf.x, dt)  # 상태 예측
        self.ekf.predict()

        # EKF 업데이트 단계
        self.ekf.H = self.jacobian_H(self.ekf.x)
        self.ekf.update(z, HJacobian=self.jacobian_H, Hx=self.observation_function)

        p_x, p_y, v_x, a_x = self.ekf.x[0], self.ekf.x[1], self.ekf.x[3], self.ekf.x[6]
        self.log_file.write(f"{p_x},{p_y}\n")

        rospy.loginfo("EKF State: p_x={:.2f}, v_y={:.2f}, a_x={:.2f}".format(p_x, v_x, a_x))

    def __del__(self):
        self.log_file.close()
        
if __name__ == '__main__':
    try:
        node = ImuEKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
