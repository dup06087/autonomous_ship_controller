import os
import json
from math import radians, cos, sin, atan2, sqrt, degrees
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
        data = [json.loads(line.strip()) for line in f]

    gps_point1 = next(item for item in data if item['time'] == time1)
    gps_point2 = next(item for item in data if item['time'] == time2)

    return gps_point1, gps_point2

# Function to extract time from the point cloud filename
def extract_time_from_filename(filename):
    time_str = filename.split('_')[-1].replace('.pcd', '')
    return float(time_str)

# Function to save results to a txt file
def save_results_to_txt(results, output_file="icp_results.txt"):
    with open(output_file, 'w') as f:
        for result in results:
            f.write(json.dumps(result) + "\n")

# Main function to calculate the GPS displacement based on point cloud filenames
def main_gnss(pcd_file1, pcd_file2, gnss_file):
    time1 = extract_time_from_filename(pcd_file1)
    time2 = extract_time_from_filename(pcd_file2)

    gps_point1, gps_point2 = load_gnss_data(gnss_file, time1, time2)

    lat1, lon1, heading1 = gps_point1['latitude'], gps_point1['longitude'], gps_point1['heading']
    lat2, lon2, heading2 = gps_point2['latitude'], gps_point2['longitude'], gps_point2.get('heading', heading1)

    gps_distance = compute_gps_displacement(lat1, lon1, lat2, lon2)
    heading_change = heading2 - heading1

    delta_x, delta_y = convert_to_local_coordinates(gps_distance, heading1, heading1)

    return {
        "file1": pcd_file1,
        "file2": pcd_file2,
        "gps_distance": gps_distance,
        "heading_change": heading_change,
        "local_displacement": {"x": delta_x, "y": delta_y}
    }

################################

# ICP Functions
def downsample(cloud, voxel_size=0.05):
    return cloud.voxel_down_sample(voxel_size)

def compute_normals(pcd, search_radius=5, max_nn=30):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=search_radius, max_nn=max_nn))
    return pcd

def crop_by_height(cloud, min_height=0):
    points = np.asarray(cloud.points)
    cropped_points = points[points[:, 2] >= min_height]
    cropped_cloud = o3d.geometry.PointCloud()
    cropped_cloud.points = o3d.utility.Vector3dVector(cropped_points)
    return cropped_cloud

def apply_point_to_plane_icp(source, target, icp_initial_guess=np.eye(4), distance_threshold=10):
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold,
        icp_initial_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=100
        )
    )
    return reg_icp

def load_point_cloud(pcd_file):
    cloud = o3d.io.read_point_cloud(pcd_file)
    cloud = downsample(cloud)
    return cloud

def main_icp(pcd_file1, pcd_file2, min_height=0):
    source = load_point_cloud(pcd_file1)
    target = load_point_cloud(pcd_file2)
    
    source_cropped = crop_by_height(source, min_height)
    target_cropped = crop_by_height(target, min_height)

    source_cropped = compute_normals(source_cropped)
    target_cropped = compute_normals(target_cropped)
    
    yaw = 0
    icp_initial_guess = np.array([
        [np.cos(radians(yaw)), -np.sin(radians(yaw)), 0, 1],
        [np.sin(radians(yaw)),  np.cos(radians(yaw)), 0, 0],
        [0,                    0,                   1, 0],
        [0,                    0,                   0, 1]
    ])
    
    reg_icp = apply_point_to_plane_icp(source_cropped, target_cropped, icp_initial_guess)
    transformation = reg_icp.transformation

    return {
        "transformation": transformation.tolist(),
        "fitness": reg_icp.fitness,
        "rmse": reg_icp.inlier_rmse
    }

################################

def process_folder_for_icp_and_gnss(folder_path, gnss_file):
    pcd_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.pcd')], key=extract_time_from_filename)

    results = []
    for i in range(len(pcd_files) - 1):
        time1 = extract_time_from_filename(pcd_files[i])
        time2 = extract_time_from_filename(pcd_files[i + 1])

        print(f"Processing files: {pcd_files[i]} and {pcd_files[i + 1]}")  # Debug print
        print(f"Time difference: {abs(time2 - time1)}")  # Debug print

        if abs(time2 - time1) <= 0.2:
            pcd_file1 = os.path.join(folder_path, pcd_files[i])
            pcd_file2 = os.path.join(folder_path, pcd_files[i + 1])

            # Run GNSS displacement calculation
            gnss_result = main_gnss(pcd_file1, pcd_file2, gnss_file)
            print(f"GNSS result: {gnss_result}")  # Debug print

            # Run ICP registration
            icp_result = main_icp(pcd_file1, pcd_file2, min_height=0)
            print(f"ICP result: {icp_result}")  # Debug print

            # Combine both results
            result = {**gnss_result, **icp_result}
            results.append(result)

    # Save results to txt file
    if results:
        save_results_to_txt(results, output_file="icp_gnss_results.txt")
        print("Results saved to icp_gnss_results.txt")  # Debug print
    else:
        print("No results to save.")  # Debug print



if __name__ == "__main__":
    folder_path = "./extracted_pointclouds"
    gnss_file = "./matched_gps_data.txt"
    process_folder_for_icp_and_gnss(folder_path, gnss_file)
