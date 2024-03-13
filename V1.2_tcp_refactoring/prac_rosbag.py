#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import rosbag

class PointCloud2Bag:
    def __init__(self, topic, bag_file_name):
        self.topic = topic
        self.bag = rosbag.Bag(bag_file_name, 'w')
        self.subscriber = rospy.Subscriber(topic, PointCloud2, self.callback)

    def callback(self, msg):
        self.bag.write(self.topic, msg, rospy.Time.now())

    def close_bag(self):
        self.bag.close()

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_rosbag', anonymous=True)
    topic_name = '/velodyne_points'  # VLP-16 포인트클라우드 데이터가 발행되는 토픽 이름
    bag_file_name = 'vlp16_data.bag'  # 저장할 rosbag 파일 이름

    pc2_bag = PointCloud2Bag(topic_name, bag_file_name)
    
    rospy.on_shutdown(pc2_bag.close_bag)  # 노드 종료 시 rosbag 파일 닫기

    rospy.spin()  # ROS 노드 실행, 콜백 함수를 통해 데이터 수신 및 저장
