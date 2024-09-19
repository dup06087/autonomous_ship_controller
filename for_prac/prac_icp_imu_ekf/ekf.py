import numpy as np
import rospy
from filterpy.kalman import ExtendedKalmanFilter
from std_msgs.msg import Float64MultiArray

class ExtendedKalmanFilterFusion:
    def __init__(self, initial_lat=0, initial_lon=0, initial_heading=0, dt=0.1, R=6378137.0):
        self.dt = dt
        self.R = R

        # 초기 위경도 및 헤딩 설정
        self.initial_lat = initial_lat
        self.initial_lon = initial_lon
        self.initial_heading = initial_heading

        self.icp_initial_guess = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        
        # EKF 설정
        self.ekf = ExtendedKalmanFilter(dim_x=10, dim_z=3)

        # 초기 상태 벡터 [lat, lon, heading, dx, dy, vx, vy, accx, accy, wz]
        self.ekf.x = np.zeros(10)
        self.ekf.x[0] = self.initial_lat     # 초기 위도 설정
        self.ekf.x[1] = self.initial_lon     # 초기 경도 설정
        self.ekf.x[2] = self.initial_heading # 초기 헤딩 설정

        # 초기 상태 공분산 행렬 (P)
        self.ekf.P *= 1.0  # 초기 상태 불확실성 (예: 대각선 요소를 1로 설정)

        # 측정 잡음 공분산 행렬 (R)
        self.ekf.R = np.eye(3) * 0.1  # 측정 잡음의 공분산

        # 프로세스 잡음 공분산 행렬 (Q)
        self.ekf.Q = np.eye(10) * 0.1  # 임의의 작은 값으로 설정

        # ROS 구독자 설정
        rospy.Subscriber('/imu/corrected', Float64MultiArray, self.imu_callback)
        rospy.Subscriber('/icp/delta', Float64MultiArray, self.icp_callback)

        # 상태 추정 결과 퍼블리시
        self.state_pub = rospy.Publisher('/kalman_filter/state', Float64MultiArray, queue_size=10)

    def state_transition_function(self, x, dt):
        lat, lon, heading, dx, dy, vx, vy, accx, accy, wz = x

        # 회전 변환 적용
        d_north = dx * np.cos(heading) - -dy * np.sin(heading)
        d_east = dx * np.sin(heading) + -dy * np.cos(heading)

        # 새로운 상태 예측
        lat_new = lat + d_north / self.R
        lon_new = lon + d_east / (self.R * np.cos(lat))
        heading_new = heading + -wz * dt

        # 속도와 가속도 업데이트
        vx_new = vx + accx * dt
        vy_new = vy + accy * dt

        # 변위 업데이트
        dx_new = dx + vx * dt
        dy_new = dy + vy * dt

        return np.array([lat_new, lon_new, heading_new, dx_new, dy_new, vx_new, vy_new, accx, accy, wz])

    def jacobian_of_transition_function(self, x, dt):
        lat, lon, heading, dx, dy, vx, vy, accx, accy, wz = x

        F = np.eye(10)  # 기본적으로 단위 행렬
        cos_heading = np.cos(heading)
        sin_heading = np.sin(heading)
        cos_lat = np.cos(lat)
        sin_lat = np.sin(lat)

        # ∂lat/∂dx, ∂lat/∂dy, ∂lat/∂heading
        F[0, 3] = cos_heading / self.R  # ∂lat/∂dx
        F[0, 4] = -sin_heading / self.R  # ∂lat/∂dy
        F[0, 2] = (-dx * sin_heading - dy * cos_heading) / self.R  # ∂lat/∂heading

        # ∂lon/∂dx, ∂lon/∂dy, ∂lon/∂heading, ∂lon/∂lat
        F[1, 3] = sin_heading / (self.R * cos_lat)  # ∂lon/∂dx
        F[1, 4] = cos_heading / (self.R * cos_lat)  # ∂lon/∂dy
        F[1, 2] = (dx * cos_heading - dy * sin_heading) / (self.R * cos_lat)  # ∂lon/∂heading
        F[1, 0] = (dx * sin_heading + dy * cos_heading) * sin_lat / (self.R * cos_lat ** 2)  # ∂lon/∂lat

        # 나머지 Jacobian 행렬 요소들
        F[2, 9] = dt  # ∂heading/∂wz
        F[3, 5] = dt  # ∂dx/∂vx
        F[4, 6] = dt  # ∂dy/∂vy
        F[5, 7] = dt  # ∂vx/∂accx
        F[6, 8] = dt  # ∂vy/∂accy

        return F

    def measurement_function(self, x):
        """ 측정 모델 """
        lat, lon, heading, dx, dy, vx, vy, accx, accy, wz = x
        return np.array([lat, lon, heading])

    def jacobian_of_measurement_function(self, x):
        """ 측정 함수의 Jacobian """
        return np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # ∂lat/∂lat
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # ∂lon/∂lon
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # ∂heading/∂heading
        ])

    def update(self, z):
        self.ekf.update(z, self.jacobian_of_measurement_function, self.measurement_function)

    def predict(self):
        # 예측 단계에서 상태 전이 함수와 Jacobian 행렬을 계산
        F = self.jacobian_of_transition_function(self.ekf.x, self.dt)
        self.ekf.F = F

        # EKF 상태 예측
        self.ekf.x = self.state_transition_function(self.ekf.x, self.dt)
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q

    def imu_callback(self, imu_data):
        accx, accy, wz = imu_data.data  # 보정된 IMU 데이터 수신
        
        # 상태 벡터 업데이트
        self.ekf.x[7] = accx  # accx 갱신
        self.ekf.x[8] = accy  # accy 갱신
        self.ekf.x[9] = wz    # wz 갱신

        # 예측 단계
        self.predict()
        print(self.ekf.x[0], self.ekf.x[1], self.ekf.x[2])
        # 상태 추정 결과 퍼블리시
        self.publish_state()

    def icp_callback(self, icp_data):
        # 측정된 Lat, Lon, Heading 데이터를 수신
        measured_lat, measured_lon, measured_heading = icp_data.data
        
        # 상태 업데이트를 위한 측정값을 구성
        z = np.array([measured_lat, measured_lon, measured_heading])
        
        # EKF 상태 업데이트
        self.update(z)
        
        # 상태 추정 결과 퍼블리시
        self.publish_state()


    def publish_state(self):
        """상태 벡터를 퍼블리시"""
        state_msg = Float64MultiArray()
        state_msg.data = self.ekf.x
        self.state_pub.publish(state_msg)

if __name__ == '__main__':
    rospy.init_node('ekf_fusion', anonymous=True)
    ekf_fusion = ExtendedKalmanFilterFusion()
    rospy.spin()
