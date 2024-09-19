import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import message_filters
from geometry_msgs.msg import Vector3Stamped  # angular velocity는 Vector3Stamped 타입

class SimpleLocalization:
    def __init__(self):
        rospy.init_node('simple_localization', anonymous=True)
        
        # Publisher for raw localization output (x, y, yaw)
        self.localization_pub = rospy.Publisher('/imu/raw_localization', Float64MultiArray, queue_size=10)
        
        # Subscribe to free acceleration and angular velocity
        free_acc_sub = message_filters.Subscriber('/filter/free_acceleration', Vector3Stamped)
        ang_vel_sub = message_filters.Subscriber('/imu/angular_velocity', Vector3Stamped)

        # Time synchronizer to sync both topics
        ts = message_filters.ApproximateTimeSynchronizer([free_acc_sub, ang_vel_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.imu_callback)

        self.previous_time = rospy.Time.now()

        # 초기 상태 (위치 x, y는 0, 초기 yaw도 0으로 설정)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # 초기 yaw (헤딩)

        # 초기 속도 (가속도를 적분하여 속도 계산)
        self.vx = 0.0
        self.vy = 0.0

    def imu_callback(self, free_acc_data, ang_vel_data):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()

        # 선형 가속도 데이터
        ax = free_acc_data.vector.x
        ay = free_acc_data.vector.y

        # 각속도 데이터 (yaw rate)
        wz = ang_vel_data.vector.z

        # yaw 업데이트 (각속도를 적분하여 yaw 계산)
        self.yaw -= wz * delta_time

        # 회전 행렬을 사용해 가속도를 전역 좌표계로 변환
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        # 가속도를 전역 좌표계로 변환
        acc_x_global = cos_yaw * ax - sin_yaw * -ay
        acc_y_global = sin_yaw * ax + cos_yaw * -ay

        # 속도를 업데이트 (가속도를 적분하여 속도를 계산)
        self.vx += acc_x_global * delta_time
        self.vy += acc_y_global * delta_time

        # 위치를 업데이트 (속도를 적분하여 위치를 계산)
        self.x += self.vx * delta_time
        self.y += self.vy * delta_time

        # 보정된 데이터 퍼블리시 (x, y, yaw 출력)
        localization_data = Float64MultiArray()
        localization_data.data = [self.x, self.y, self.yaw * 180 / math.pi]  # yaw을 degree로 변환하여 출력
        self.localization_pub.publish(localization_data)

        self.previous_time = current_time

if __name__ == '__main__':
    try:
        simple_localization = SimpleLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
