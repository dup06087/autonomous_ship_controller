#!/usr/bin/env python2
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def handle_odom_msg(msg):
    global broadcaster
    
    # 현재 메시지의 타임스탬프와 프레임 설정
    timestamp = rospy.Time.now()
    
    # 메시지에서 받은 위치 정보 설정
    translation = (msg.pose.pose.position.x,
                   msg.pose.pose.position.y,
                   msg.pose.pose.position.z)

    # 메시지에서 받은 방향 정보 설정
    quaternion = (msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w)
    
    # 변환 정보 발행
    transform = TransformStamped()
    transform.header.stamp = timestamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    
    broadcaster.sendTransform(transform)

def odom_listener():
    global broadcaster
    rospy.init_node('odom_to_tf_broadcaster')
    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/odom', Odometry, handle_odom_msg)
    rospy.spin()  # 노드가 계속 실행되도록 유지

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
