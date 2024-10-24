import open3d as o3d
import numpy as np
from math import radians, cos, sin, atan2, degrees

# Function to downsample point cloud for faster processing
def downsample(cloud, voxel_size=0.05):
    return cloud.voxel_down_sample(voxel_size)

# Function to compute normals using legacy Open3D
def compute_normals(pcd, search_radius=1.0, max_nn=30):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=search_radius, max_nn=max_nn))
    return pcd

# Function to crop point cloud by height
def crop_by_height(cloud, min_height=0):
    points = np.asarray(cloud.points)
    cropped_points = points[points[:, 2] >= min_height]
    cropped_cloud = o3d.geometry.PointCloud()
    cropped_cloud.points = o3d.utility.Vector3dVector(cropped_points)
    return cropped_cloud

# Function to extract FPFH features for RANSAC
def compute_fpfh(pcd, voxel_size):
    radius_normal = voxel_size * 2
    radius_feature = voxel_size * 5

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return fpfh

# Execute RANSAC-based global registration
def execute_ransac_icp(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    
    return result

# Load the point cloud from the file
def load_point_cloud(pcd_file):
    cloud = o3d.io.read_point_cloud(pcd_file)
    return downsample(cloud)

# Visualize the registration result with legacy point clouds and origin marker
def visualize_icp_result(source_cloud, target_cloud, transformation):
    source_transformed = source_cloud.transform(transformation)
    source_transformed.paint_uniform_color([1, 0.706, 0])  # Yellow for transformed source
    target_cloud.paint_uniform_color([0, 0.651, 0.929])    # Blue for target

    origin_marker = create_origin_marker(radius=2.5)
    o3d.visualization.draw_geometries([source_transformed, target_cloud, origin_marker])

# Function to create a small sphere at the origin for visualization
def create_origin_marker(radius=0.1):
    origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
    origin_sphere.paint_uniform_color([1, 0, 0])
    origin_sphere.translate([0, 0, 0])
    return origin_sphere

# Function to extract rotation and translation from the transformation matrix
def print_transformation_details(transformation):
    rotation_angle = atan2(transformation[1, 0], transformation[0, 0])
    rotation_angle_degrees = degrees(rotation_angle)
    
    translation_x = transformation[0, 3]
    translation_y = transformation[1, 3]

    print(f"Rotation (degrees): {rotation_angle_degrees:.2f}")
    print(f"Translation: x = {translation_x:.2f}, y = {translation_y:.2f}")

# Main function
def main(pcd_file1, pcd_file2, voxel_size=0.05, min_height=0):
    source = load_point_cloud(pcd_file1)
    target = load_point_cloud(pcd_file2)
    
    source_cropped = crop_by_height(source, min_height)
    target_cropped = crop_by_height(target, min_height)

    source_fpfh = compute_fpfh(source_cropped, voxel_size)
    target_fpfh = compute_fpfh(target_cropped, voxel_size)
    
    result_ransac = execute_ransac_icp(source_cropped, target_cropped, source_fpfh, target_fpfh, voxel_size)
    
    transformation = result_ransac.transformation
    print(transformation)
    print_transformation_details(transformation)

    visualize_icp_result(source_cropped, target_cropped, transformation)

if __name__ == "__main__":
    pcd_file1 = "./extracted_pointclouds/pointcloud_1724857705.0.pcd"
    pcd_file2 = "./extracted_pointclouds/pointcloud_1724857705.2.pcd"
    
    main(pcd_file1, pcd_file2, voxel_size=0.05, min_height=0)
