#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

def publish_odometry():
    rospy.init_node('odometry_publisher')
    
    # Odometry 정보를 발행할 Publisher 생성
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    rate = rospy.Rate(5)  # 10Hz
    
    while not rospy.is_shutdown():
        # Odometry 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # 위치 정보 설정 (여기서는 0,0,0에 위치)
        odom_msg.pose.pose.position.x = 0
        odom_msg.pose.pose.position.y = 0
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 1
        
        # 속도 정보 설정 (여기서는 모든 속도를 0으로 설정)
        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0
        
        # 메시지 발행
        odom_pub.publish(odom_msg)
        
        rate.sleep()

        print("publishing")
        
if __name__ == '__main__':
    try:
        publish_odometry()

    except rospy.ROSInterruptException:
        print("error end")