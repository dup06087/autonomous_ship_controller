#!/usr/bin/env python3

import rospy
import rosbag
from rospy import ROSInterruptException

class AllTopicsToBag:
    def __init__(self, bag_file_name):
        self.bag = rosbag.Bag(bag_file_name, 'w')
        self.subscribers = []  # 구독자 목록을 저장하는 리스트

    def subscribe_to_all_topics(self):
        topics = rospy.get_published_topics()
        for topic, datatype in topics:
            sub = rospy.Subscriber(topic, rospy.AnyMsg, self.callback, callback_args=topic)
            self.subscribers.append(sub)
            print(f"Subscribed to {topic}")

    def callback(self, msg, topic):
        self.bag.write(topic, msg, rospy.Time.now())

    def close_bag(self):
        self.bag.close()
        print("Rosbag closed.")

if __name__ == '__main__':
    rospy.init_node('all_topics_to_rosbag', anonymous=True)
    bag_file_name = 'all_data.bag'  # 저장할 rosbag 파일 이름
    all_topics_bag = AllTopicsToBag(bag_file_name)

    try:
        all_topics_bag.subscribe_to_all_topics()
        rospy.spin()
    except ROSInterruptException:
        pass
    finally:
        all_topics_bag.close_bag()