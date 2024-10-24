import open3d as o3d
import numpy as np
from math import radians, cos, sin, atan2, degrees

# Function to downsample point cloud for faster processing
def downsample(cloud, voxel_size=0.05):
    return cloud.voxel_down_sample(voxel_size)

# Function to compute normals using legacy Open3D
def compute_normals(pcd, search_radius=5, max_nn=30):
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

# Function to apply Point-to-Plane ICP between two point clouds using legacy API
def apply_point_to_plane_icp(source, target, icp_initial_guess=np.eye(4), distance_threshold=1.5):
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
def main(pcd_file1, pcd_file2, min_height=0):
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
        [np.cos(radians(yaw)), -np.sin(radians(yaw)), 0, 1],
        [np.sin(radians(yaw)),  np.cos(radians(yaw)), 0, 0],
        [0,                    0,                   1, 0],
        [0,                    0,                   0, 1]
    ])
    
    # Perform Point-to-Plane ICP registration
    reg_icp = apply_point_to_plane_icp(source_cropped, target_cropped, icp_initial_guess)
    transformation = reg_icp.transformation
    print(transformation)
    print('done before visualization')

    # Print the transformation details
    print_transformation_details(transformation)

    # Visualize the registration result
    visualize_icp_result(source_cropped, target_cropped, transformation)

if __name__ == "__main__":
    # Input point cloud files
    # pcd_file1 = "./extracted_pointclouds/pointcloud_1724857727.8.pcd"
    # pcd_file2 = "./extracted_pointclouds/pointcloud_1724857730.6.pcd"
    pcd_file1 = "./extracted_pointclouds/pointcloud_1724857704.0.pcd"
    pcd_file2 = "./extracted_pointclouds/pointcloud_1724857707.2.pcd"
    
    # Run the main function with optional cropping by height (min_height = 0)
    main(pcd_file1, pcd_file2, min_height=0)
