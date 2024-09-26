#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion

def publish_odometry():
    rospy.init_node('odometry_publisher')
    
    # Odometry 정보를 발행할 Publisher 생성
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz
    
    theta = 0.0  # 시작 각도
    radius = 20.0  # 원의 반지름 (미터)
    angular_speed = 0.1  # 각속도 (라디안/초)

    while not rospy.is_shutdown():
        # x, y 위치 계산
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        
        # 각도 업데이트
        theta += angular_speed / 10.0  # 10Hz 주기로 각속도 적용

        # Odometry 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_lnik"
        
        # 위치 정보 설정
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        # 속도 정보 설정
        odom_msg.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular_speed))
        
        # 메시지 발행
        odom_pub.publish(odom_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odometry()
    except rospy.ROSInterruptException:
        pass
