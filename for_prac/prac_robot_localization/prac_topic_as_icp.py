#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import random
import threading

def euler_to_quaternion(roll, pitch, yaw):
    """
    Euler angles (roll, pitch, yaw) to quaternion conversion.
    This method calculates the quaternion from the given Euler angles.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return Quaternion(qx, qy, qz, qw)

def publish_imu_data():
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_frame'

        # Orientation (roll, pitch, yaw) -> random values for testing
        roll = random.uniform(-math.pi, math.pi)
        pitch = random.uniform(-math.pi, math.pi)
        yaw = random.uniform(-math.pi, math.pi)

        # Convert Euler angles to quaternion
        imu_msg.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Angular velocity (yaw rate)
        imu_msg.angular_velocity.z = random.uniform(-0.1, 0.1)  # yaw rate

        # Linear acceleration (just an example)
        imu_msg.linear_acceleration.x = random.uniform(-1, 1)
        imu_msg.linear_acceleration.y = random.uniform(-1, 1)

        # Publish the message
        imu_pub.publish(imu_msg)
        rate.sleep()

def publish_icp_data():
    odom_pub = rospy.Publisher('/icp/odometry', Odometry, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz (slower rate for ICP simulation)

    x = 0.0
    y = 0.0
    theta = 0.0

    while not rospy.is_shutdown():
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'

        # Simulating ICP-based odometry
        delta_x = random.uniform(-0.01, 0.01)  # random small movement
        delta_y = random.uniform(-0.01, 0.01)  # random small movement
        delta_theta = random.uniform(-0.01, 0.01)  # random small rotation

        x += delta_x
        y += delta_y
        theta += delta_theta

        # Pose (position and orientation)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y

        # Convert Euler angles to quaternion
        odom_msg.pose.pose.orientation = euler_to_quaternion(0, 0, theta)

        # Twist (linear and angular velocities)
        odom_msg.twist.twist.linear.x = delta_x * 10  # simulating velocity
        odom_msg.twist.twist.linear.y = delta_y * 10
        odom_msg.twist.twist.angular.z = delta_theta * 10  # simulating angular velocity (yaw)

        # Publish the message
        odom_pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 메인 스레드에서 노드 초기화
        rospy.init_node('imu_icp_publisher_node', anonymous=True)

        # 스레드를 사용하여 두 퍼블리셔를 동시에 실행
        imu_thread = threading.Thread(target=publish_imu_data)
        icp_thread = threading.Thread(target=publish_icp_data)

        # 스레드 실행
        imu_thread.start()
        icp_thread.start()

        # 스레드가 종료될 때까지 대기
        imu_thread.join()
        icp_thread.join()

    except rospy.ROSInterruptException:
        pass
