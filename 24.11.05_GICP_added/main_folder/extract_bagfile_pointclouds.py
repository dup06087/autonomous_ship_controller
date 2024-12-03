import rosbag
import os
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import numpy as np

# Load the filtered closest timestamps from the file
def load_filtered_timestamps(file_path):
    timestamps = {}
    with open(file_path, 'r') as f:
        for line in f:
            # Parse the line to extract both GPS and Bag timestamps
            parts = line.split(',')
            gps_time_str = parts[0].split(':')[1].strip()
            gps_time = float(gps_time_str)

            bag_time_str = parts[1].split(':')[1].strip()
            bag_time = float(bag_time_str)

            # Store the mapping between GPS and bag time
            timestamps[bag_time] = gps_time
    return timestamps

# Extract PointCloud2 data for the given timestamps and save with GPS time
def extract_pointclouds_from_bag(bag_file_path, timestamp_map, output_dir):
    bag = rosbag.Bag(bag_file_path, 'r')
    extracted_count = 0
    
    # Iterate through the bag file and find messages that match the timestamps
    for topic, msg, t in bag.read_messages(topics=['/velodyne_points']):  # Adjust topic name if needed
        timestamp = t.to_sec()
        
        # Check if this timestamp is in our list of filtered bag timestamps
        if timestamp in timestamp_map:
            gps_time = timestamp_map[timestamp]
            print(f"Extracting point cloud with GPS time: {gps_time}")
            
            # Convert the PointCloud2 message to Open3D point cloud object
            pc = convert_pointcloud2_to_open3d(msg)
            
            # Save the point cloud to a file using GPS time
            file_name = os.path.join(output_dir, f"pointcloud_{gps_time}.pcd")
            o3d.io.write_point_cloud(file_name, pc)
            print(f"Saved point cloud to {file_name}")
            
            extracted_count += 1
            
    bag.close()
    print(f"Total extracted point clouds: {extracted_count}")

# Helper function to convert PointCloud2 to Open3D point cloud
def convert_pointcloud2_to_open3d(ros_point_cloud):
    # Read points from the PointCloud2 message
    points = np.array([p[:3] for p in pc2.read_points(ros_point_cloud, field_names=("x", "y", "z"), skip_nans=True)])
    
    # Create Open3D PointCloud object
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(points)
    
    return o3d_cloud

# Define paths
bag_file_path = './pc_data_2024_11_20_1047.bag'
filtered_timestamps_file = './filtered_closest_timestamps_lake_one_cycle.txt'
output_dir = './extracted_pointclouds_lake_one_cycle'
    
# Ensure output directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Load the filtered timestamps (mapping Bag time to GPS time)
filtered_timestamps_map = load_filtered_timestamps(filtered_timestamps_file)

# Extract and save point clouds from the ROS bag file using GPS time as filename
extract_pointclouds_from_bag(bag_file_path, filtered_timestamps_map, output_dir)
