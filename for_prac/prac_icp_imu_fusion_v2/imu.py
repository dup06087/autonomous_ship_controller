import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.previous_time = rospy.Time.now()

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
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
    
    def imu_callback(self, data):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.previous_time).to_sec()

        # 쿼터니언에서 RPY(roll, pitch, yaw) 구하기
        orientation = data.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # 선형 가속도 보정
        linear_acceleration = data.linear_acceleration
        corrected_acc = self.rotate_acceleration(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z, roll, pitch, yaw)

        # 각속도 보정
        angular_velocity = data.angular_velocity
        corrected_angular_velocity = self.rotate_angular_velocity(angular_velocity.x, angular_velocity.y, angular_velocity.z, roll, pitch, yaw)

        # Corrected accx, accy, wz 퍼블리시
        corrected_data = Float64MultiArray()
        corrected_data.data = [corrected_acc[0], corrected_acc[1], corrected_angular_velocity[2]]
        self.correction_pub.publish(corrected_data)

        self.previous_time = current_time

if __name__ == '__main__':
    try:
        rospy.init_node('imu_corrector', anonymous=True)
        imu_corrector = IMUCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
