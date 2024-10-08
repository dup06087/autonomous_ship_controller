import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ros_numpy import point_cloud2 as ros_np
import open3d as o3d
import open3d.core as o3c
import time

class PointCloudProcessor:
    def __init__(self):
        self.sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.pub = rospy.Publisher("/processed_pointcloud", PointCloud2, queue_size=10)
        self.vff_force = -2.0
        self.voxel_size = 0.05
        self.pitch = 0

    def crop_roi(self, pcd, start, end):
        min_bound = o3c.Tensor(start, dtype=o3c.float32, device=pcd.device)
        max_bound = o3c.Tensor(end, dtype=o3c.float32, device=pcd.device)
        roi_bounding_box = o3d.t.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        return pcd.crop(roi_bounding_box)

    def voxel_down_sampling(self, pcd, voxel_size):
        return pcd.voxel_down_sample(voxel_size)

    def rotate_point_cloud_by_pitch(self, pcd):
        if self.pitch == 0:
            return pcd
        pitch_rad = np.radians(-self.pitch)
        R = o3c.Tensor([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                        [0, 1, 0],
                        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]], dtype=o3c.float32, device=pcd.device)
        return pcd.transform(R)

    def remove_ship_body(self, pcd, ship_body_bounds):
        # GPU에서 CPU로 데이터를 이동
        pcd_tensor = pcd.point.positions.to(o3c.Device("CPU:0"))

        # ship_body_bounds의 min과 max 값을 텐서로 변환
        min_bound = np.array(ship_body_bounds['min'])
        max_bound = np.array(ship_body_bounds['max'])

        # 범위에 포함되지 않는 포인트를 남김
        mask = ~((pcd_tensor[:, 0].numpy() >= min_bound[0]) &
                (pcd_tensor[:, 0].numpy() <= max_bound[0]) &
                (pcd_tensor[:, 1].numpy() >= min_bound[1]) &
                (pcd_tensor[:, 1].numpy() <= max_bound[1]) &
                (pcd_tensor[:, 2].numpy() >= min_bound[2]) &
                (pcd_tensor[:, 2].numpy() <= max_bound[2]))

        filtered_pcd_tensor = pcd_tensor.numpy()[mask]
        # 필터링된 포인트들을 GPU로 다시 전송
        filtered_pcd = o3d.t.geometry.PointCloud(o3c.Tensor(filtered_pcd_tensor, dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return filtered_pcd


    def callback(self, msg):
        time_prev = time.time()
        time_diff = rospy.Time.now() - msg.header.stamp
        if time_diff.to_sec() > 0.05:
            return
        pc_array = ros_np.pointcloud2_to_array(msg)
        points = np.column_stack((pc_array['x'], pc_array['y'], pc_array['z']))
        pcd = o3d.t.geometry.PointCloud(o3c.Tensor(points, dtype=o3c.float32, device=o3c.Device("CUDA:0")))

        pcd = self.crop_roi(pcd, start=[-10, -10, self.vff_force], end=[10, 10, 0.2])

        pcd = self.voxel_down_sampling(pcd, voxel_size=self.voxel_size)

        pcd = self.rotate_point_cloud_by_pitch(pcd)

        ship_body_bounds = {'min': [-1.1, -1, -0.6], 'max': [1.1, 1, 0.31]}
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
        rospy.loginfo("Published processed point cloud. Processing Time: {:.2f} seconds".format(time.time() - time_prev))

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pointcloud_processor", anonymous=True)
    processor = PointCloudProcessor()
    processor.run()
