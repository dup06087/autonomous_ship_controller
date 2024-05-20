#!/usr/bin/env python

import rospy
import threading
from obstacle_layer_odom_filter.msg import gps_data  # 사용자 정의 메시지 임포트

class GPSPublisher:
    def __init__(self):
        rospy.init_node('gps_data_publisher', anonymous=True)
        self.gps_pub = rospy.Publisher('/gps/data', gps_data, queue_size=10)
        self.lock = threading.Lock()
        self.current_gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'heading': 0.0
        }

    def update_gps_data(self, new_data):
        with self.lock:
            self.current_gps_data.update(new_data)

    def publish_gps_data(self):
        rate = rospy.Rate(5)  # 1 Hz
        while not rospy.is_shutdown():
            with self.lock:
                gps_msg = gps_data()
                gps_msg.latitude = self.current_gps_data['latitude']
                gps_msg.longitude = self.current_gps_data['longitude']
                gps_msg.heading = self.current_gps_data['heading']
                self.gps_pub.publish(gps_msg)
                rospy.loginfo("Published GPS data: %s", gps_msg)
            rate.sleep()

    def start(self):
        publish_thread = threading.Thread(target=self.publish_gps_data)
        publish_thread.start()

if __name__ == '__main__':
    try:
        gps_publisher = GPSPublisher()
        gps_publisher.start()

        # Simulate GPS data updates from different threads
        def simulate_gps_data_updates():
            import time
            import random
            while not rospy.is_shutdown():
                new_data = {
                    'latitude': random.uniform(37.0000001, 37.0000005),
                    'longitude': random.uniform(127.0000001, 127.0000005),
                    'heading': random.uniform(0, 360)
                }
                gps_publisher.update_gps_data(new_data)
                time.sleep(0.2)

        simulation_thread = threading.Thread(target=simulate_gps_data_updates)
        simulation_thread.start()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
