#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import datetime
import threading

# Global variable to keep track of the last received message time
last_msg_time = None
is_topic_ok = True

def callback(msg):
    global last_msg_time
    global is_topic_ok
    # Update the last message time when a new message is received
    last_msg_time = rospy.get_time()
    is_topic_ok = True

def monitor_topic():
    global last_msg_time
    global is_topic_ok
    rate = rospy.Rate(1)  # Check the topic every second
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        # Check if new message received within the last 5 seconds
        if last_msg_time is not None and current_time - last_msg_time > 5:
            rospy.logwarn("No data from VLP-16 for 5 seconds at {}".format(
                datetime.datetime.now()))
            is_topic_ok = False
        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('vlp16_monitor', anonymous=True)
        
        # Subscribe to the VLP-16 point cloud topic
        rospy.Subscriber("/velodyne_points", PointCloud2, callback)
        
        # Start the topic monitoring in a separate thread
        monitor_thread = threading.Thread(target=monitor_topic)
        monitor_thread.start()
        
        # Keep the main thread alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass