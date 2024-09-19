import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from std_msgs.msg import Float64MultiArray

class KalmanFilterFusion:
    def __init__(self, dt=0.1, R=6378137.0):
        self.dt = dt  # 시간 간격
        self.R = R    # 지구 반지름

        # 칼만 필터 설정
        self.kf = KalmanFilter(dim_x=8, dim_z=3)

        # 상태 벡터 초기화 [dx, dy, vx, vy, accx, accy, heading_diff, wz]
        self.kf.x = np.zeros(8)

        # 상태 전이 행렬 (F)
        self.kf.F = np.array([
            [1, 0, dt,  0, 0.5*dt**2, 0, 0, 0],  # dx += vx * dt + 0.5 * accx * dt^2
            [0, 1, 0, dt, 0, 0.5*dt**2, 0, 0],   # dy += vy * dt + 0.5 * accy * dt^2
            [0, 0, 1,  0, dt, 0, 0, 0],          # vx += accx * dt
            [0, 0, 0,  1, 0, dt, 0, 0],          # vy += accy * dt
            [0, 0, 0,  0, 1, 0, 0, 0],           # accx 유지 (측정값)
            [0, 0, 0,  0, 0, 1, 0, 0],           # accy 유지 (측정값)
            [0, 0, 0,  0, 0, 0, 1, -dt],         # heading_diff -= wz * dt
            [0, 0, 0,  0, 0, 0, 0, 1]            # wz 유지 (측정값)
        ])

        # 측정 행렬 (H)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],  # dx 측정
            [0, 1, 0, 0, 0, 0, 0, 0],  # dy 측정
            [0, 0, 0, 0, 0, 0, 1, 0]   # heading_diff 측정
        ])

        # 측정 잡음 공분산 행렬 (R)
        self.kf.R = np.eye(3) * 0.1  # 임의의 작은 값으로 설정

        # 프로세스 잡음 공분산 행렬 (Q)
        self.kf.Q = np.eye(8) * 0.1  # 임의의 작은 값으로 설정

        # 초기 상태 공분산 행렬 (P)
        self.kf.P = np.eye(8) * 1  # 초기 상태 불확실성

        # ROS 구독자 설정
        rospy.Subscriber('/imu/corrected', Float64MultiArray, self.imu_callback)
        rospy.Subscriber('/icp/delta', Float64MultiArray, self.icp_callback)

        # 상태 추정 결과 퍼블리시
        self.state_pub = rospy.Publisher('/kalman_filter/state', Float64MultiArray, queue_size=10)

    def imu_callback(self, imu_data):
        """보정된 IMU 데이터를 사용하여 상태 예측"""
        accx, accy, wz = imu_data.data  # 보정된 IMU 데이터 수신
        
        # 상태 벡터 업데이트 (보정된 IMU 데이터를 반영)
        self.kf.x[4] = accx  # accx 갱신
        self.kf.x[5] = accy  # accy 갱신
        self.kf.x[7] = wz    # wz 갱신

        # 예측 단계
        self.kf.predict()

        # 상태 추정 결과 퍼블리시 (필요시)
        self.publish_state()

    def icp_callback(self, icp_data):
        """ICP 결과로 상태 업데이트"""
        delta_x, delta_y, heading_diff = icp_data.data  # ICP 데이터 수신

        # ICP 측정값으로 상태 업데이트
        z = np.array([delta_x, delta_y, heading_diff])
        self.kf.update(z)

        # 상태 추정 결과 퍼블리시 (필요시)
        self.publish_state()
        print(self.kf.x)
        
    def publish_state(self):
        """상태 벡터를 퍼블리시"""
        state_msg = Float64MultiArray()
        state_msg.data = self.kf.x
        self.state_pub.publish(state_msg)

    def calculate_lat_lon_heading(self, lat_prev, lon_prev):
        """후처리 단계에서 최종 위도, 경도, 헤딩 계산"""
        filtered_dx, filtered_dy, filtered_heading_diff = self.kf.x[0], self.kf.x[1], self.kf.x[6]
        heading_rad = np.radians(filtered_heading_diff)

        # 회전 변환 적용
        d_north = filtered_dx * np.cos(heading_rad) - filtered_dy * np.sin(heading_rad)
        d_east = filtered_dx * np.sin(heading_rad) + filtered_dy * np.cos(heading_rad)

        # 위도 경도 계산
        delta_lat = d_north / self.R
        delta_lon = d_east / (self.R * np.cos(np.radians(lat_prev)))

        lat_new = lat_prev + np.degrees(delta_lat)
        lon_new = lon_prev + np.degrees(delta_lon)

        return lat_new, lon_new, filtered_heading_diff

if __name__ == '__main__':
    rospy.init_node('kalman_filter_fusion', anonymous=True)
    kf_fusion = KalmanFilterFusion()
    rospy.spin()
