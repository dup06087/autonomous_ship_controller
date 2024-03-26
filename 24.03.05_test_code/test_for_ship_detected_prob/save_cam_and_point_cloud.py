#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
import rosbag
from datetime import datetime

class DataToBag:
    def __init__(self, pc_topic, cam_topic):
        self.pc_topic = pc_topic
        self.cam_topic = cam_topic
        self.start_time = datetime.now()
        self.bag_file_name = self.generate_bag_filename(self.start_time)
        self.bag = rosbag.Bag(self.bag_file_name, 'w')
        self.pc_subscriber = rospy.Subscriber(pc_topic, PointCloud2, self.pc_callback)
        self.cam_subscriber = rospy.Subscriber(cam_topic, Image, self.cam_callback)

    def generate_bag_filename(self, start_time):
        filename = start_time.strftime("data_%Y_%m_%d_%H%M.bag")
        return filename

    def pc_callback(self, msg):
        self.bag.write(self.pc_topic, msg, rospy.Time.now())

    def cam_callback(self, msg):
        self.bag.write(self.cam_topic, msg, rospy.Time.now())

    def close_bag(self):
        self.bag.close()
        print(f"Bag closed. Filename: {self.bag_file_name}")

if __name__ == '__main__':
    rospy.init_node('data_to_rosbag', anonymous=True)
    pc_topic_name = '/velodyne_points'  # 포인트클라우드 데이터 토픽
    cam_topic_name = '/usb_cam/image_raw'  # 카메라 데이터 토픽

    data_bag = DataToBag(pc_topic_name, cam_topic_name)
    
    rospy.on_shutdown(data_bag.close_bag)

    rospy.spin()
