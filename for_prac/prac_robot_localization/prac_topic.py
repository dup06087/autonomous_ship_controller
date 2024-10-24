#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import random
import math

def publish_imu():
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    imu_msg = Imu()

    # 임시 IMU 데이터 설정 (orientation 및 angular velocity만 설정)
    imu_msg.orientation.x = random.uniform(-1.0, 1.0)
    imu_msg.orientation.y = random.uniform(-1.0, 1.0)
    imu_msg.orientation.z = random.uniform(-1.0, 1.0)
    imu_msg.orientation.w = random.uniform(-1.0, 1.0)

    imu_msg.angular_velocity.z = random.uniform(-0.1, 0.1)  # Yaw 각속도 (임의 값)

    imu_pub.publish(imu_msg)

def publish_gps():
    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    gps_msg = NavSatFix()

    # 임시 GPS 데이터 설정 (latitude, longitude, 그리고 heading)
    gps_msg.latitude = random.uniform(37.0, 38.0)   # 임의 위도 값
    gps_msg.longitude = random.uniform(127.0, 128.0) # 임의 경도 값

    # Heading(absolute yaw)은 별도로 처리할 수 있지만 이 예제에서는 간단히 설명
    # 실제 GPS에서 헤딩을 얻으려면 센서가 헤딩 값을 포함해야 함

    gps_pub.publish(gps_msg)

if __name__ == '__main__':
    rospy.init_node('imu_gps_publisher')

    rate = rospy.Rate(10)  # 10Hz로 퍼블리시

    while not rospy.is_shutdown():
        publish_imu()
        publish_gps()
        rate.sleep()
