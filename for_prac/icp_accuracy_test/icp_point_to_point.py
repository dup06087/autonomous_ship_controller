import open3d as o3d
import numpy as np
from math import radians, cos, sin, atan2, degrees

# Function to downsample point cloud for faster processing
def downsample(cloud, voxel_size=0.05):
    return cloud.voxel_down_sample(voxel_size)

# Function to crop point cloud by height
def crop_by_height(cloud, min_height=0):
    cloud_legacy = cloud.to_legacy()
    points = np.asarray(cloud_legacy.points)
    cropped_points = points[points[:, 2] >= min_height]
    cropped_cloud_legacy = o3d.geometry.PointCloud()
    cropped_cloud_legacy.points = o3d.utility.Vector3dVector(cropped_points)
    cropped_cloud_tensor = o3d.t.geometry.PointCloud.from_legacy(cropped_cloud_legacy)
    return cropped_cloud_tensor

# Function to apply Point-to-Point ICP between two point clouds
def apply_point_to_point_icp(source, target, icp_initial_guess=np.eye(4), distance_threshold=1.5):
    reg_icp = o3d.t.pipelines.registration.icp(
        source,
        target,
        max_correspondence_distance=distance_threshold,
        init_source_to_target=icp_initial_guess,
        estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-20,
            relative_rmse=1e-20,
            max_iteration=1000
        )
    )
    return reg_icp

# Load the point cloud from the file
def load_point_cloud(pcd_file):
    cloud = o3d.t.io.read_point_cloud(pcd_file)
    cloud = downsample(cloud)
    return cloud

# Visualize the registration result with legacy point clouds and origin marker
def visualize_icp_result(source_cloud, target_cloud, transformation):
    source_transformed = source_cloud.transform(transformation)
    source_transformed_cpu = source_transformed.to_legacy()
    target_cloud_cpu = target_cloud.to_legacy()

    source_transformed_cpu.paint_uniform_color([1, 0.706, 0])  # Yellow for transformed source
    target_cloud_cpu.paint_uniform_color([0, 0.651, 0.929])    # Blue for target

    # Add the origin marker
    origin_marker = create_origin_marker(radius=2.5)  # You can also use create_coordinate_frame()

    # Visualize the registration and marker
    o3d.visualization.draw_geometries([source_transformed_cpu, target_cloud_cpu, origin_marker])

# Function to create a small sphere at the origin
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
    source = load_point_cloud(pcd_file1)
    target = load_point_cloud(pcd_file2)
    
    source_cropped = crop_by_height(source, min_height)
    target_cropped = crop_by_height(target, min_height)
    
    yaw = 0
    
    icp_initial_guess = np.array([
        [np.cos(radians(yaw)), -np.sin(radians(yaw)), 0, 0],
        [np.sin(radians(yaw)), np.cos(radians(yaw)),  0, 0],
        [0,                   0,                   1, 0],
        [0,                   0,                   0, 1]
    ])
    reg_icp = apply_point_to_point_icp(source_cropped, target_cropped, icp_initial_guess)
    transformation = reg_icp.transformation.cpu().numpy()
    print(transformation)
    print('done before visualization')

    print_transformation_details(transformation)

    visualize_icp_result(source_cropped, target_cropped, transformation)

if __name__ == "__main__":
    pcd_file1 = "./extracted_pointclouds/pointcloud_1724857705.0.pcd"
    pcd_file2 = "./extracted_pointclouds/pointcloud_1724857705.2.pcd"
    
    main(pcd_file1, pcd_file2, min_height=0)
