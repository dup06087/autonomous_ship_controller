import os
import json
from math import radians, cos, sin, atan2, sqrt, degrees, hypot
import open3d as o3d
import numpy as np

# Constants
EARTH_RADIUS = 6371000  # Earth radius in meters

# Function to compute the displacement from latitude and longitude using Haversine formula
def compute_gps_displacement(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = EARTH_RADIUS * c  # Distance in meters
    return distance

# Function to compute bearing between two GPS points
def compute_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    
    return degrees(bearing) % 360  # Normalize to 0-360 degrees

# Function to convert displacement to local x, y coordinates using initial heading
def convert_to_local_coordinates(distance, bearing, initial_heading):
    bearing_rad = radians(bearing)
    initial_heading_rad = radians(initial_heading)

    delta_x = distance * cos(bearing_rad - initial_heading_rad)
    delta_y = distance * sin(bearing_rad - initial_heading_rad)

    return delta_x, delta_y

# Function to read GNSS data from the file and match by time
def load_gnss_data(gnss_file, time1, time2):
    with open(gnss_file, 'r') as f:
        data = []
        for line in f:
            line = line.strip()
            if line:  # Check if the line is not empty
                try:
                    json_line = json.loads(line)
                    data.append(json_line)
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON on line: {line}")
                    raise e  # Optionally, you can handle it better based on your needs

    gps_point1 = next(item for item in data if item['time'] == time1)
    gps_point2 = next(item for item in data if item['time'] == time2)

    return gps_point1, gps_point2

# Function to compute FPFH features for RANSAC
def compute_fpfh(pcd, voxel_size):
    radius_normal = voxel_size * 1.5  # 기본 2에서 줄임
    radius_feature = voxel_size * 3   # 기본 5에서 줄임

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30))
    
    return fpfh

# RANSAC Function with optimized parameters
def apply_ransac_icp(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.0  # 줄여서 속도 향상
    
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.8),  # 완화
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=10, confidence=0.95))  # 반복 횟수 줄임
    
    return result_ransac


# Load the point cloud from the file
def load_point_cloud(pcd_file):
    cloud = o3d.io.read_point_cloud(pcd_file)
    cloud = cloud.voxel_down_sample(voxel_size=0.05)
    return cloud

# Function to extract transformation details
def extract_transformation_details(transformation):
    delta_x = -transformation[0, 3]
    delta_y = transformation[1, 3]
    yaw_change = degrees(atan2(transformation[1, 0], transformation[0, 0]))
    icp_distance = hypot(delta_x, delta_y)
    return delta_x, delta_y, yaw_change, icp_distance

# Process point cloud sequence with RANSAC and GNSS comparison
def process_sequence_with_gnss_and_ransac(folder_path, gnss_file):
    pcd_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.pcd')], key=extract_time_from_filename)
    
    results = []
    voxel_size = 0.05  # Set the voxel size for downsampling and FPFH calculation
    
    for i in range(len(pcd_files) - 1):
        pcd_file1 = os.path.join(folder_path, pcd_files[i])
        pcd_file2 = os.path.join(folder_path, pcd_files[i + 1])

        # GNSS displacement
        gnss_result = main_gnss(pcd_file1, pcd_file2, gnss_file)

        # Load and downsample point clouds
        source = load_point_cloud(pcd_file1)
        target = load_point_cloud(pcd_file2)

        # Compute FPFH features for both point clouds
        source_fpfh = compute_fpfh(source, voxel_size)
        target_fpfh = compute_fpfh(target, voxel_size)

        # RANSAC registration
        reg_ransac = apply_ransac_icp(source, target, source_fpfh, target_fpfh, voxel_size)

        # Extract transformation details
        transformation = reg_ransac.transformation
        delta_x, delta_y, yaw_change, icp_distance = extract_transformation_details(transformation)
        
        # Combine GNSS and ICP results into the flat format
        result = {
            "gps_distance": gnss_result["gps_distance"],
            "x_change_gnss": gnss_result["x_change_gnss"],
            "y_change_gnss": gnss_result["y_change_gnss"],
            "heading_change_gnss": gnss_result["heading_change_gnss"],
            "icp_distance": icp_distance,
            "x_change_icp": delta_x,
            "y_change_icp": delta_y,
            "yaw_change_icp": yaw_change,
            "fitness": reg_ransac.fitness,
            "rmse": reg_ransac.inlier_rmse
        }
        results.append(result)
        
        # Print results for each pair
        print(f"Processing pair {i+1}: {pcd_files[i]} and {pcd_files[i+1]}")
        print(result)
    
    # Save the results to a file in the required format
    save_results_to_txt(results, output_file="ransac_gnss_results.txt")

# Main function to calculate the GPS displacement based on point cloud filenames
def main_gnss(pcd_file1, pcd_file2, gnss_file):
    time1 = extract_time_from_filename(pcd_file1)
    time2 = extract_time_from_filename(pcd_file2)

    gps_point1, gps_point2 = load_gnss_data(gnss_file, time1, time2)

    lat1, lon1, heading1 = gps_point1['latitude'], gps_point1['longitude'], gps_point1['heading']
    lat2, lon2, heading2 = gps_point2['latitude'], gps_point2['longitude'], gps_point2.get('heading', heading1)

    gps_distance = compute_gps_displacement(lat1, lon1, lat2, lon2)
    bearing = compute_bearing(lat1, lon1, lat2, lon2)
    heading_change = heading2 - heading1

    delta_x, delta_y = convert_to_local_coordinates(gps_distance, bearing, heading1)

    return {
        "gps_distance": gps_distance,
        "x_change_gnss": delta_x,
        "y_change_gnss": delta_y,
        "heading_change_gnss": heading_change
    }

# Function to extract time from the point cloud filename
def extract_time_from_filename(filename):
    time_str = filename.split('_')[-1].replace('.pcd', '')
    return float(time_str)

# Function to save results to a txt file
def save_results_to_txt(results, output_file="ransac_gnss_results.txt"):
    with open(output_file, 'w') as f:
        for result in results:
            f.write(json.dumps(result) + "\n")

if __name__ == "__main__":
    folder_path = "./extracted_pointclouds"
    gnss_file = "./matched_gps_data.txt"
    process_sequence_with_gnss_and_ransac(folder_path, gnss_file)
