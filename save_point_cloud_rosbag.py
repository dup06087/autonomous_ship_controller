#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import rosbag
from datetime import datetime
import os

class PointCloud2Bag:
    def __init__(self, topic):
        self.topic = topic
        self.start_time = datetime.now()  # 시작 시간 기록
        self.folder_path = self.create_log_folder(self.start_time)
        self.bag_file_name = self.generate_bag_filename(self.start_time, self.folder_path)
        self.bag = rosbag.Bag(self.bag_file_name, 'w')
        self.subscriber = rospy.Subscriber(topic, PointCloud2, self.callback)

    def create_log_folder(self, start_time):
        # Create folder path based on the current year, month-date, and time
        year = start_time.strftime("%Y")
        month_date = start_time.strftime("%m-%d")
        time = start_time.strftime("%H%M")
        folder_path = os.path.join("log_pc", year, month_date, time)
        
        # Ensure the folder exists
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        
        return folder_path

    def generate_bag_filename(self, start_time, folder_path):
        # 파일명 형식: pc_data_년_월_일_시간.bag
        # 시간 포맷은 HHMM (시간과 분)으로 지정합니다.
        filename = start_time.strftime("pc_data_%Y_%m_%d_%H%M.bag")
        return os.path.join(folder_path, filename)

    def callback(self, msg):
        self.bag.write(self.topic, msg, rospy.Time.now())

    def close_bag(self):
        self.bag.close()
        print(f"Bag closed. Filename: {self.bag_file_name}")

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_rosbag', anonymous=True)
    topic_name = '/velodyne_points'  # VLP-16 포인트클라우드 데이터가 발행되는 토픽 이름

    pc2_bag = PointCloud2Bag(topic_name)
    
    rospy.on_shutdown(pc2_bag.close_bag)  # 노드 종료 시 rosbag 파일 닫기

    rospy.spin()  # ROS 노드 실행, 콜백 함수를 통해 데이터 수신 및 저장
