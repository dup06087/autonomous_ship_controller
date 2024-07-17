#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odom_msg(msg):
    br = tf.TransformBroadcaster()
    
    # 메시지에서 받은 위치 및 방향 정보를 사용하여 변환 정보 발행
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

def odom_listener():
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.Subscriber('/odom', Odometry, handle_odom_msg)
    rospy.spin()  # 노드가 계속 실행되도록 유지

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
