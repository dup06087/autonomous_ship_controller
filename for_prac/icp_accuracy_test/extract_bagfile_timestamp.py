import rosbag

# Path to your ROS bag file
bag_file_path = '/home/ices/Desktop/rosbag_for_fusion/rect1.bag'

# Topic to extract timestamps from
topic_name = '/velodyne_points'  # Assuming this is the topic for the point cloud data

def extract_timestamps_from_bag(bag_file, topic):
    timestamps = []
    
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            timestamps.append(t.to_sec())  # Extract the timestamp in seconds
    
    return timestamps

def save_timestamps_to_txt(timestamps, output_file):
    with open(output_file, 'w') as f:
        for timestamp in timestamps:
            f.write(f"{timestamp}\n")  # Write each timestamp on a new line

# Extract timestamps
timestamps = extract_timestamps_from_bag(bag_file_path, topic_name)

# Save timestamps to a text file
output_file = './rect1_timestamp.txt'
save_timestamps_to_txt(timestamps, output_file)

print(f"Timestamps saved to {output_file}")
