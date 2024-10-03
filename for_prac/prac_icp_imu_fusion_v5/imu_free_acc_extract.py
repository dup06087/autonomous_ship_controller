import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.previous_time = rospy.Time.now()

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
        delta_time = (current_time - self.previous_time).to_sec()

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

        # Corrected free acceleration과 w_z 퍼블리시
        corrected_data = Float64MultiArray()
        corrected_data.data = [free_acc[0], free_acc[1], w_z]  # free_acc_x, free_acc_y, w_z
        self.correction_pub.publish(corrected_data)

        self.previous_time = current_time

if __name__ == '__main__':
    try:
        rospy.init_node('imu_corrector', anonymous=True)
        imu_corrector = IMUCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
