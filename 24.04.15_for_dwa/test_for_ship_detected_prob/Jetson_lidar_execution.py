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
        self.pitch = None
        
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
    
    def filter_by_intensity(self, pc_arr, threshold=5):
        mask = pc_arr['intensity'] > threshold
        return pc_arr[mask]

    def crop_roi(self, pcd, start, end):
        # start=[-0.924, -0.36, -0.7]; end=[0.96, 0.36, 0.2] # no obstacle, little bigger than crop roi 
        # start=[-1, -5, -0.5]; end=[20, 5, 0.5] # original
        
        min_bound = np.array(start)
        max_bound = np.array(end)
        roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        return pcd.crop(roi_bounding_box)
    

    def rotate_point_cloud_by_pitch(self, pcd):
        """
        Rotate the point cloud by a given pitch angle.

        :param pcd: Open3D point cloud object to be rotated.
        :param pitch: The pitch angle in degrees to rotate the point cloud.
        :return: Rotated Open3D point cloud object.
        """
        
        if self.pitch == None:
            return pcd
        
        pitch_rad = np.radians(self.pitch)
        
        R = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                      [0, 1, 0],
                      [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
        
        # Apply the rotation to each point in the point cloud
        pcd.rotate(R, center=(0, 0, 0))
        
        return pcd
    
    def callback(self, msg):
        time_diff = rospy.Time.now() - msg.header.stamp
        if time_diff.to_sec() > 0.05: # realtime
            return
        
        pc_array = ros_np.pointcloud2_to_array(msg)
        # pc_array = self.filter_by_intensity(pc_array, threshold=self.intensity)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack((pc_array['x'], pc_array['y'], pc_array['z'])))

        pcd = self.voxel_down_sampling(pcd, voxel_size=self.voxel_size)

        pcd = self.rotate_point_cloud_by_pitch(pcd)
        
        pcd = self.crop_roi(pcd, start=[-100, -100, -0.6], end=[100, 100, 0.3])

        # ship_body_bounds = {'min': [-1.1, -0.51, -0.6], 'max': [1.1, 0.51, 0.1]}  # 선체가 위치하는 영역을 지정
        ship_body_bounds = {'min': [-1.1, -1, -0.6], 'max': [1.1, 1, 0.31]}  # 선체가 위치하는 영역을 지정
        pcd = self.remove_ship_body(pcd, ship_body_bounds)
        
        # Flatten the z-coordinate to create a 2D point cloud
        points = np.asarray(pcd.points)
        points[:, 2] = 0  # Set z values to 0
        pcd.points = o3d.utility.Vector3dVector(points)
        
        pcd, ind = self.radius_outlier_removal(pcd, nb_points=int(self.dbscan_minpoints), radius=self.dbscan_eps)
        
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

