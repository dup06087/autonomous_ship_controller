import json
from math import radians, cos, sin, atan2, sqrt, degrees
import open3d as o3d
import numpy as np
from math import radians, cos, sin, atan2, degrees

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
    # Convert bearing and heading to radians
    bearing_rad = radians(bearing)
    initial_heading_rad = radians(initial_heading)

    # Calculate x and y displacements in the local coordinate system
    delta_x = distance * cos(bearing_rad - initial_heading_rad)
    delta_y = distance * sin(bearing_rad - initial_heading_rad)

    return delta_x, delta_y

# Function to read GNSS data from the file and match by time
def load_gnss_data(gnss_file, time1, time2):
    with open(gnss_file, 'r') as f:
        data = [json.loads(line.strip()) for line in f]

    # Find the GNSS entries that match the times
    gps_point1 = next(item for item in data if item['time'] == time1)
    gps_point2 = next(item for item in data if item['time'] == time2)

    return gps_point1, gps_point2

# Function to extract time from the point cloud filename
def extract_time_from_filename(filename):
    time_str = filename.split('_')[-1].replace('.pcd', '')  # Extract time string
    return float(time_str)  # Convert to float

# Main function to calculate the GPS displacement based on point cloud filenames
def main(pcd_file1, pcd_file2, gnss_file):
    # Extract times from the point cloud filenames
    time1 = extract_time_from_filename(pcd_file1)
    time2 = extract_time_from_filename(pcd_file2)

    print(f"Extracted times: {time1}, {time2}")

    # Load the corresponding GNSS data for the two times
    gps_point1, gps_point2 = load_gnss_data(gnss_file, time1, time2)

    # Extract latitude, longitude, and heading from both points
    lat1, lon1, heading1 = gps_point1['latitude'], gps_point1['longitude'], gps_point1['heading']
    lat2, lon2, heading2 = gps_point2['latitude'], gps_point2['longitude'], gps_point2.get('heading', heading1)

    # Compute GPS displacement (distance)
    gps_distance = compute_gps_displacement(lat1, lon1, lat2, lon2)

    # Calculate heading change
    heading_change = heading2 - heading1

    # Convert GPS displacement into local coordinates based on the initial heading
    delta_x, delta_y = convert_to_local_coordinates(gps_distance, heading1, heading1)  # Using heading1 for reference

    # Output the results
    print(f"Point 1 (lat, lon, heading): ({lat1}, {lon1}, {heading1})")
    print(f"Point 2 (lat, lon, heading): ({lat2}, {lon2}, {heading2})")
    print(f"GPS Displacement: {gps_distance:.2f} meters")
    print(f"Heading Change: {heading_change:.2f} degrees")
    print(f"Local Displacement: x = {delta_x:.2f} meters, y = {delta_y:.2f} meters (relative to initial heading)")


################################

# Function to downsample point cloud for faster processing
def downsample(cloud, voxel_size=0.05):
    return cloud.voxel_down_sample(voxel_size)

# Function to compute normals using legacy Open3D
def compute_normals(pcd, search_radius=1.0, max_nn=30):
    # Estimate normals using legacy API
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=search_radius, max_nn=max_nn))
    return pcd  # Returns the same point cloud with normals computed

# Function to crop point cloud by height
def crop_by_height(cloud, min_height=0):
    points = np.asarray(cloud.points)
    cropped_points = points[points[:, 2] >= min_height]  # Crop points above min_height (z-axis)
    cropped_cloud = o3d.geometry.PointCloud()
    cropped_cloud.points = o3d.utility.Vector3dVector(cropped_points)
    return cropped_cloud  # Return legacy format

# Function to apply Generalized ICP between two point clouds using legacy API
def apply_gicp_legacy(source, target, icp_initial_guess=np.eye(4), distance_threshold=1.5):
    reg_gicp = o3d.pipelines.registration.registration_generalized_icp(
        source, target, distance_threshold,
        icp_initial_guess,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=1000
        )
    )
    return reg_gicp

# Load the point cloud from the file
def load_point_cloud(pcd_file):
    cloud = o3d.io.read_point_cloud(pcd_file)  # Load in legacy format
    cloud = downsample(cloud)  # Optional: downsample the point cloud for efficiency
    return cloud

# Visualize the registration result with legacy point clouds and origin marker
def visualize_icp_result(source_cloud, target_cloud, transformation):
    source_transformed = source_cloud.transform(transformation)
    source_transformed.paint_uniform_color([1, 0.706, 0])  # Yellow for transformed source
    target_cloud.paint_uniform_color([0, 0.651, 0.929])    # Blue for target

    # Add the origin marker (for visualizing the starting point)
    origin_marker = create_origin_marker(radius=2.5)

    # Visualize the registration and marker
    o3d.visualization.draw_geometries([source_transformed, target_cloud, origin_marker])

# Function to create a small sphere at the origin for visualization
def create_origin_marker(radius=0.1):
    origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
    origin_sphere.paint_uniform_color([1, 0, 0])  # Red color for the origin marker
    origin_sphere.translate([0, 0, 0])  # Place it at the origin
    return origin_sphere

# Function to extract rotation and translation from the transformation matrix
def print_transformation_details(transformation):
    # Extract rotation angle (in radians) and convert to degrees
    rotation_angle = atan2(transformation[1, 0], transformation[0, 0])  # atan2(sin, cos)
    rotation_angle_degrees = degrees(rotation_angle)
    
    # Extract translation in x and y
    translation_x = transformation[0, 3]
    translation_y = transformation[1, 3]

    # Print the results
    print(f"Rotation (degrees): {rotation_angle_degrees:.2f}")
    print(f"Translation: x = {translation_x:.2f}, y = {translation_y:.2f}")

# Main function
def main_gicp(pcd_file1, pcd_file2, min_height=0):
    # Load and process the point clouds
    source = load_point_cloud(pcd_file1)
    target = load_point_cloud(pcd_file2)
    
    # Crop the point clouds by height (z-axis)
    source_cropped = crop_by_height(source, min_height)
    target_cropped = crop_by_height(target, min_height)

    # Compute normals for better registration accuracy
    source_cropped = compute_normals(source_cropped)
    target_cropped = compute_normals(target_cropped)
    
    # Define the initial guess (optional, can be from IMU or fixed to identity)
    yaw = 0  # Replace with IMU heading if available
    icp_initial_guess = np.array([
        [np.cos(radians(yaw)), -np.sin(radians(yaw)), 0, 0],
        [np.sin(radians(yaw)),  np.cos(radians(yaw)), 0, 0],
        [0,                    0,                   1, 0],
        [0,                    0,                   0, 1]
    ])
    
    # Perform Generalized ICP registration
    reg_gicp = apply_gicp_legacy(source_cropped, target_cropped, icp_initial_guess)
    transformation = reg_gicp.transformation
    print(transformation)
    print('done before visualization')

    # Print the transformation details
    print_transformation_details(transformation)

    # Visualize the registration result
    visualize_icp_result(source_cropped, target_cropped, transformation)

if __name__ == "__main__":
    pcd_file1 = "./extracted_pointclouds/pointcloud_1724857705.0.pcd"
    pcd_file2 = "./extracted_pointclouds/pointcloud_1724857705.6.pcd"
    gnss_file = "./matched_gps_data.txt"  # Path to your GNSS data file
    main(pcd_file1, pcd_file2, gnss_file)
    
    main_gicp(pcd_file1, pcd_file2, min_height=0)