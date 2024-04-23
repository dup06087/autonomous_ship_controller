import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ros_numpy import point_cloud2 as ros_np
import open3d as o3d
import time 

class PointCloudProcessor:
    def __init__(self):
        # rospy.init_node("pointcloud_processor", anonymous=True)
        self.sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.pub = rospy.Publisher("/processed_pointcloud", PointCloud2, queue_size=10)
        self.bbox_lists = []
        
    def voxel_down_sampling(self, pcd, voxel_size):
        return pcd.voxel_down_sample(voxel_size)

    def radius_outlier_removal(self, pcd, nb_points, radius):
        # Open3D's remove_radius_outlier returns a tuple: (PointCloud, indices)
        filtered_pcd, ind = pcd.remove_radius_outlier(nb_points, radius)
        return filtered_pcd, ind

    def update_coeff(self, coeff_kf, coeff_kd, voxel_size, intensity, dbscan_eps, dbscan_minpoints, vff_force):
        self.coeff_kf = coeff_kf
        self.coeff_kd = coeff_kd
        self.voxel_size = voxel_size
        self.intensity = intensity
        self.dbscan_eps= dbscan_eps
        self.dbscan_minpoints = dbscan_minpoints
        self.vff_force = vff_force
        
    def remove_ship_body(self, pcd, ship_body_bounds):
        """
        Remove the ship body from the point cloud data.

        :param pcd: The input point cloud as an Open3D point cloud object.
        :param ship_body_bounds: A dictionary with 'min' and 'max' keys indicating the bounding box to remove.
                                Format: {'min': [x_min, y_min, z_min], 'max': [x_max, y_max, z_max]}
        :return: Filtered point cloud with the ship body removed.
        """
        # Convert to numpy array for easier manipulation
        pcd_np = np.asarray(pcd.points)

        # Apply conditions to filter out points within the ship body bounds
        condition = ~((pcd_np[:, 0] >= ship_body_bounds['min'][0]) & (pcd_np[:, 0] <= ship_body_bounds['max'][0]) &
                    (pcd_np[:, 1] >= ship_body_bounds['min'][1]) & (pcd_np[:, 1] <= ship_body_bounds['max'][1]) &
                    (pcd_np[:, 2] >= ship_body_bounds['min'][2]) & (pcd_np[:, 2] <= ship_body_bounds['max'][2]))

        # Filter points based on the condition
        filtered_pcd_np = pcd_np[condition]

        # Create a new Open3D point cloud object from the filtered points
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_pcd_np)

        return filtered_pcd


    def callback(self, msg):
        time_diff = rospy.Time.now() - msg.header.stamp
        if time_diff.to_sec() > 0.05: # realtime
            return
        
        pc_array = ros_np.pointcloud2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack((pc_array['x'], pc_array['y'], pc_array['z'])))

        # start=[-0.924, -0.36, -0.7]; end=[0.96, 0.36, 0.2] # no obstacle, little bigger than crop roi 
        # start=[-1, -5, -0.5]; end=[20, 5, 0.5] # original
        start=[-100, -100, -0.7]; end=[100, 100, 0.5] 
        
        min_bound = np.array(start)
        max_bound = np.array(end)
        roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        pcd = pcd.crop(roi_bounding_box)

        ship_body_bounds = {'min': [-1, -0.4, -0.6], 'max': [1, 0.4, 0.1]}  # 선체가 위치하는 영역을 지정
        pcd = self.remove_ship_body(pcd, ship_body_bounds)
        
        # Voxel down-sampling
        pcd = self.voxel_down_sampling(pcd, voxel_size=0.05)

        # Radius outlier removal
        pcd, ind = self.radius_outlier_removal(pcd, nb_points=10, radius=0.7)
        if len(pcd.points) == 0:
            rospy.logwarn("No points left after filtering")
            return

        # Ensure the point cloud is not empty
        points = np.asarray(pcd.points)
        if points.shape[0] == 0:
            rospy.loginfo("Processed point cloud has no points.")
            return

        # Convert Open3D PointCloud back to ROS PointCloud2
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id =  'velodyne'
        points_xyz = np.asarray(pcd.points)
        points_xyz = pc2.create_cloud_xyz32(header, points_xyz)
        # processed_msg = ros_np.array_to_pointcloud2(points, header)

        self.pub.publish(points_xyz)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pointcloud_processor", anonymous=True)

    processor = PointCloudProcessor()

    processor.run()

