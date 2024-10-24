import csv
import rosbag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Open the bag file
bag = rosbag.Bag('./2024-10-22-16-50-14.bag')

# Open a CSV file to write
with open('odometry_filtered.csv', 'w') as csvfile:
    csv_writer = csv.writer(csvfile)

    # Write the header
    csv_writer.writerow(['time', 'position_x', 'position_y', 'position_z',
                         'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                         'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                         'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'])

    # Read the messages from the odometry/filtered topic
    for topic, msg, t in bag.read_messages(topics=['/odometry/filtered']):
        # Write each message as a row in the CSV
        csv_writer.writerow([t.to_sec(),
                             msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                             msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                             msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                             msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

# Close the bag
bag.close()