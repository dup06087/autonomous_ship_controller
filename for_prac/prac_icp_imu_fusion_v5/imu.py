import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

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
        self.prev_heading = 0  # Initialize previous heading
        self.prev_time = None

    def quaternion_to_euler(self, x, y, z, w):
        # 쿼터니언을 오일러 각도로 변환 (Roll, Pitch, Yaw 계산)
        rotation = R.from_quat([x, y, z, w])
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)  # 라디안 단위
        return roll, pitch, yaw

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

    def imu_callback(self, data):
        current_time = rospy.Time.now()

        # 첫 번째 콜백인 경우, 이전 시간 및 상태 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return  # Skip the first iteration since we can't compute delta_time yet

        # Calculate delta time between IMU callbacks
        delta_time = (current_time - self.prev_time).to_sec()

        # 쿼터니언에서 Roll, Pitch, Yaw 계산
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

        # 각속도(w_z)를 그대로 사용 (2D 평면에서의 z축 각속도)
        angular_velocity = data.angular_velocity
        w_z = angular_velocity.z  # z축 각속도

        # Heading update: z축 각속도(w_z)를 이용하여 헤딩 업데이트
        self.dheading_step = w_z * delta_time * 180 / math.pi  # 각속도를 각도로 변환
        self.dheading += self.dheading_step
        current_heading = (self.prev_heading - self.dheading_step) % 360
        heading_rad = math.radians(current_heading)

        # 속도 업데이트: a * t로부터 속도 업데이트
        self.vx += free_acc[0] * delta_time
        self.vy += free_acc[1] * delta_time

        # 로컬 좌표계에서 전역 좌표계로 속도 변환
        global_v_north = self.vx * math.cos(heading_rad) - -self.vy * math.sin(heading_rad)
        global_v_east = self.vx * math.sin(heading_rad) + -self.vy * math.cos(heading_rad)

        # 위치 업데이트 (전역 좌표계)
        self.dnorth += global_v_north * delta_time
        self.deast += global_v_east * delta_time

        # Corrected free acceleration과 w_z 퍼블리시
        corrected_data = Float64MultiArray()
        corrected_data.data = [free_acc[0], free_acc[1], w_z]  # free_acc_x, free_acc_y, w_z
        self.correction_pub.publish(corrected_data)

        # 이전 시간 및 헤딩 업데이트
        self.prev_heading = current_heading
        self.prev_time = current_time

    def pre_icp_reset(self):
        """Reset only the IMU deltas before ICP starts, keep velocities intact."""
        self.dnorth = 0
        self.deast = 0
        self.dheading = 0
    
    def post_icp_reset(self, vx, vy):
        """Reset both deltas and velocities after ICP has completed."""
        self.vx = vx
        self.vy = vy

if __name__ == '__main__':
    try:
        rospy.init_node('imu_corrector', anonymous=True)
        imu_corrector = IMUCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
