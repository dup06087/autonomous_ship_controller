#!/usr/bin/env python
import rospy
import threading
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import random


# Euler angles (roll, pitch, yaw) to quaternion conversion
def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return Quaternion(qx, qy, qz, qw)


def publish_imu_data():
    imu_pub = rospy.Publisher('/example/imu', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'base_link'

        # Random orientation (roll, pitch, yaw) for testing
        roll = random.uniform(-math.pi, math.pi)
        pitch = random.uniform(-math.pi, math.pi)
        yaw = random.uniform(-math.pi, math.pi)

        imu_msg.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Angular velocity (yaw rate)
        imu_msg.angular_velocity.z = random.uniform(-0.1, 0.1)  # Yaw angular velocity

        # Linear acceleration
        imu_msg.linear_acceleration.x = random.uniform(-1, 1)
        imu_msg.linear_acceleration.y = random.uniform(-1, 1)

        # Publish the message
        imu_pub.publish(imu_msg)
        rate.sleep()


def publish_odometry_data():
    odom_pub = rospy.Publisher('/example/odom', Odometry, queue_size=10)
    rate = rospy.Rate(30)  # 30 Hz, to match the `frequency` set in the YAML file

    x = 0.0
    y = 0.0
    theta = 0.0

    while not rospy.is_shutdown():
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'

        # Simulating random odometry data (x, y, theta)
        delta_x = random.uniform(-0.01, 0.01)
        delta_y = random.uniform(-0.01, 0.01)
        delta_theta = random.uniform(-0.01, 0.01)

        x += delta_x
        y += delta_y
        theta += delta_theta

        # Pose (position and orientation)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = euler_to_quaternion(0, 0, theta)

        # Twist (linear and angular velocities)
        odom_msg.twist.twist.linear.x = delta_x * 10
        odom_msg.twist.twist.linear.y = delta_y * 10
        odom_msg.twist.twist.angular.z = delta_theta * 10

        # Publish the message
        odom_pub.publish(odom_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('sensor_publisher_node', anonymous=True)

        # Create threads for IMU and Odometry publishers
        imu_thread = threading.Thread(target=publish_imu_data)
        odom_thread = threading.Thread(target=publish_odometry_data)

        # Start both threads
        imu_thread.start()
        odom_thread.start()

        # Join threads to wait for their completion
        imu_thread.join()
        odom_thread.join()

    except rospy.ROSInterruptException:
        pass
