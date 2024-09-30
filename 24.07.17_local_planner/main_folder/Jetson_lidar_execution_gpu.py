import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ros_numpy import point_cloud2 as ros_np
import open3d as o3d
import open3d.core as o3c
import time
from threading import Lock

class PointCloudProcessor:
    def __init__(self):
        self.sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback, queue_size=2)
        self.pub = rospy.Publisher("/processed_pointcloud", PointCloud2, queue_size=2)
        self.bbox_lists = []
        self.pitch = None
        self.vff_force = 0.2
        self.voxel_size = 0.1
        self.lock = Lock()

    def update_coeff(self, coeff_kv_p, coeff_kv_i, coeff_kv_d, coeff_kw_p, coeff_kw_i, coeff_kw_d, voxel_size, intensity, dbscan_eps, dbscan_minpoints, vff_force):
        self.coeff_kv_p = coeff_kv_p
        self.coeff_kv_i = coeff_kv_i
        self.coeff_kv_d = coeff_kv_d
        self.coeff_kw_p = coeff_kw_p
        self.coeff_kw_i = coeff_kw_i
        self.coeff_kw_d = coeff_kw_d
        self.voxel_size = voxel_size
        self.intensity = intensity
        self.dbscan_eps= dbscan_eps
        self.dbscan_minpoints = dbscan_minpoints
        self.vff_force = vff_force
        
    def crop_roi(self, pcd, start, end):
        min_bound = o3c.Tensor(start, dtype=o3c.float32, device=pcd.device)
        max_bound = o3c.Tensor(end, dtype=o3c.float32, device=pcd.device)
        roi_bounding_box = o3d.t.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        return pcd.crop(roi_bounding_box)

    def voxel_down_sampling(self, pcd, voxel_size):
        return pcd.voxel_down_sample(voxel_size)

    def rotate_point_cloud_by_pitch(self, pcd):
        if self.pitch is None:
            return pcd
        pitch_rad = np.radians(-self.pitch)
        R = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                    [0, 1, 0],
                    [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]], dtype=np.float32)
        
        # 3x3 회전 행렬을 4x4 변환 행렬로 확장
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R

        # Open3D의 Tensor로 변환
        T_tensor = o3c.Tensor(T, dtype=o3c.float32, device=pcd.device)

        return pcd.transform(T_tensor)

    def remove_ship_body(self, pcd, ship_body_bounds):
        min_bound = o3c.Tensor(ship_body_bounds['min'], dtype=o3c.float32, device=pcd.device)
        max_bound = o3c.Tensor(ship_body_bounds['max'], dtype=o3c.float32, device=pcd.device)

        mask = (pcd.point.positions[:, 0] < min_bound[0]) | (pcd.point.positions[:, 0] > max_bound[0]) | \
               (pcd.point.positions[:, 1] < min_bound[1]) | (pcd.point.positions[:, 1] > max_bound[1]) | \
               (pcd.point.positions[:, 2] < min_bound[2]) | (pcd.point.positions[:, 2] > max_bound[2])

        filtered_pcd = pcd.select_by_mask(mask)
        return filtered_pcd
    
    def callback(self, msg):
        # return
        time_prev = time.time()
        time_diff = rospy.Time.now() - msg.header.stamp
        if time_diff.to_sec() > 0.1:
            # print("returned lidar callback : ", time_diff.to_sec())
            return
        # return
        pc_array = ros_np.pointcloud2_to_array(msg)
        points = np.column_stack((pc_array['x'], pc_array['y'], pc_array['z']))
        pcd = o3d.t.geometry.PointCloud(o3c.Tensor(points, dtype=o3c.float32, device=o3c.Device("CUDA:0")))

        pcd = self.crop_roi(pcd, start=[-15, -15, self.vff_force], end=[15, 15, 0.2])

        pcd = self.voxel_down_sampling(pcd, voxel_size=self.voxel_size)

        pcd = self.rotate_point_cloud_by_pitch(pcd)

        # ship_body_bounds = {'min': [-1.1, -1, -0.6], 'max': [1.1, 1, 0.31]}
        ship_body_bounds = {'min': [-5, -5, -2], 'max': [5, 5, 2]}
        pcd = self.remove_ship_body(pcd, ship_body_bounds)

        points_cpu = pcd.point.positions.to(o3c.Device("CPU:0")).numpy()
        if points_cpu.shape[0] == 0:
            rospy.loginfo("Processed point cloud has no points.")
            return

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id
        points_xyz = pc2.create_cloud_xyz32(header, points_cpu)
        self.pub.publish(points_xyz)
        # rospy.loginfo("Published processed point cloud. Processing Time: {:.2f} seconds".format(time.time() - time_prev))
        
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pointcloud_processor", anonymous=True)
    processor = PointCloudProcessor()
    processor.run()
