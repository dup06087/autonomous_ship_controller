#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import rosbag
from datetime import datetime

class CameraDataToBag:
    def __init__(self, cam_topic):
        self.cam_topic = cam_topic
        self.start_time = datetime.now()
        self.bag_file_name = self.generate_bag_filename(self.start_time)
        self.bag = rosbag.Bag(self.bag_file_name, 'w')
        self.cam_subscriber = rospy.Subscriber(cam_topic, Image, self.cam_callback)

    def generate_bag_filename(self, start_time):
        filename = start_time.strftime("camera_data_%Y_%m_%d_%H%M.bag")
        return filename

    def cam_callback(self, msg):
        self.bag.write(self.cam_topic, msg, rospy.Time.now())

    def close_bag(self):
        self.bag.close()
        print(f"Bag closed. Filename: {self.bag_file_name}")

if __name__ == '__main__':
    rospy.init_node('camera_data_to_rosbag', anonymous=True)
    cam_topic_name = '/usb_cam/image_raw'  # 카메라 데이터 토픽

    cam_data_bag = CameraDataToBag(cam_topic_name)
    
    rospy.on_shutdown(cam_data_bag.close_bag)

    rospy.spin()
