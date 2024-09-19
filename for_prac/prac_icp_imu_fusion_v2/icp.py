#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import open3d.core as o3c
import math
import time

class ICPTest:
    def __init__(self, mother_instance=None):
        self.prev_scan = None
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.icp_pub = rospy.Publisher('/icp/delta', Float64MultiArray, queue_size=10)

        self.current_heading = 0.0  # Heading in degrees
        self.prev_data_time = rospy.Time.now()

    def lidar_callback(self, data):
        prev_time = time.time()

        # Check if the timestamp is too old
        time_diff = rospy.Time.now() - data.header.stamp
        if time_diff.to_sec() > 0.1:  # Adjust this threshold as needed
            return
        
        try:
            cloud = self.point_cloud2_to_o3d(data)

            # Define ROI (e.g., a box from (-10, -10, -1) to (10, 10, 1))
            min_bound = np.array([-10, -10, -1])
            max_bound = np.array([10, 10, 1])
            
            # Crop ROI
            cloud = self.crop_roi(cloud, min_bound, max_bound)

            if self.prev_scan is not None:
                # Perform ICP on GPU
                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud, 
                    target=self.prev_scan,
                    max_correspondence_distance=0.5,
                    init_source_to_target=o3c.Tensor(np.identity(4), dtype=o3c.float32, device=cloud.device),
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                )
                transf = reg_gicp.transformation.cpu().numpy()  # Convert the result back to a numpy array

                if reg_gicp.fitness > 0.8:  # Check fitness score
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    heading_change = np.degrees(rotation_euler[2])  # Yaw change in degrees
                    heading_change = math.trunc(heading_change * 10) / 10

                    # Calculate delta x, delta y based on the ICP translation
                    delta_x = translation[0]
                    delta_y = translation[1]
                    wz = heading_change / (data.header.stamp - self.prev_data_time).to_sec()

                    # Publish the results
                    icp_data = Float64MultiArray()
                    icp_data.data = [delta_x, delta_y, wz]
                    self.icp_pub.publish(icp_data)
                    
                    print(delta_x, delta_y, wz)
                else: 
                    print("ICP fitness low, ignoring this scan.")
                    
            self.prev_scan = cloud
        
        except Exception as e:
            print('ICP exception error : ', e)
        
        self.prev_data_time = data.header.stamp
        
        # print("ICP time consuming : ", time.time()-prev_time)

    def crop_roi(self, cloud, min_bound, max_bound):
        # GPU에서 ROI 자르기 수행
        min_bound_tensor = o3c.Tensor(min_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        max_bound_tensor = o3c.Tensor(max_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        bbox = o3d.t.geometry.AxisAlignedBoundingBox(min_bound_tensor, max_bound_tensor)
        cropped_cloud = cloud.crop(bbox)
        return cropped_cloud

    def point_cloud2_to_o3d(self, cloud_msg):
        points = np.array(list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))))
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(points, dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return cloud

    def rotation_matrix_to_euler(self, rotation_matrix):
        R = np.array(rotation_matrix)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])  # roll
            y = np.arctan2(-R[2, 0], sy)  # pitch
            z = np.arctan2(R[1, 0], R[0, 0])  # yaw
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])  # roll
            y = np.arctan2(-R[2, 0], sy)  # pitch
            z = 0  # yaw
        return np.array([x, y, z])

if __name__ == '__main__':
    rospy.init_node("icp_processor", anonymous=True)
    icp_test = ICPTest()
    rospy.spin()
