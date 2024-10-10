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
        self.sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.msg_callback, queue_size=1)
        self.pub = rospy.Publisher("/processed_pointcloud", PointCloud2, queue_size=1)
        self.latest_msg = None
        self.bbox_lists = []
        self.pitch = None
        self.vff_force = 0.2
        self.voxel_size = 0.1
        self.lock = Lock()

        # Timer 추가: 100ms마다 최신 메시지 처리
        rospy.Timer(rospy.Duration(0.2), self.process_latest_msg)

        # CUDA 장치 설정
        self.device = o3c.Device("CUDA:0")  # CUDA 장치 0을 사용
        self.warm_up_open3d_cuda()
        
    def warm_up_open3d_cuda(self):
        print("Warming up CUDA using Open3D-Core...")

        # 간단한 텐서를 생성하고 CUDA 디바이스에 올림
        tensor_a = o3c.Tensor(np.random.rand(100).astype(np.float32), device=self.device)
        tensor_b = o3c.Tensor(np.random.rand(100).astype(np.float32), device=self.device)

        # 두 텐서를 더하는 간단한 연산을 수행하여 CUDA 연산을 실행
        tensor_c = tensor_a + tensor_b

        # 결과 확인을 위해 CPU로 다시 복사
        result = tensor_c.cpu().numpy()
        
        print("CUDA warm-up lidar processor complete.")
        return result

    def update_coeff(self, coeff_kv_p, coeff_kv_i, coeff_kv_d, coeff_kw_p, coeff_kw_i, coeff_kw_d, voxel_size, intensity, dbscan_eps, dbscan_minpoints, vff_force):
        try:
            self.coeff_kv_p = coeff_kv_p
            self.coeff_kv_i = coeff_kv_i
            self.coeff_kv_d = coeff_kv_d
            self.coeff_kw_p = coeff_kw_p
            self.coeff_kw_i = coeff_kw_i
            self.coeff_kw_d = coeff_kw_d
            self.voxel_size = voxel_size
            self.intensity = intensity
            self.dbscan_eps = dbscan_eps
            self.dbscan_minpoints = dbscan_minpoints
            self.vff_force = vff_force
        except Exception as e:
            print("(pointcloudprocessor) Update coeff error: ", e)

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
        R = np.array([[np.cos(pitch_rad), 0, -np.sin(pitch_rad)],
                      [0, 1, 0],
                      [np.sin(pitch_rad), 0, np.cos(pitch_rad)]], dtype=np.float32)

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

    def msg_callback(self, msg):
        # 새로운 메시지를 처리하기 위한 콜백
        time_diff = rospy.Time.now() - msg.header.stamp
        if time_diff.to_sec() > 0.12:
            # print("Returned lidar callback: ", time_diff.to_sec())
            return
        
        # Lock을 걸고 최신 메시지를 저장
        with self.lock:
            self.latest_msg = msg

    def process_latest_msg(self, event):
        # 최신 메시지를 처리
        with self.lock:
            if self.latest_msg is None:
                return
            msg_copy = self.latest_msg
            self.latest_msg = None  # 처리 후 메시지 초기화

        # 이제 Lock이 풀린 상태에서 메시지 처리
        try:
            pc_array = ros_np.pointcloud2_to_array(msg_copy)
            points = np.column_stack((pc_array['x'], pc_array['y'], pc_array['z']))
            pcd = o3d.t.geometry.PointCloud(o3c.Tensor(points, dtype=o3c.float32, device=o3c.Device("CUDA:0")))

            pcd = self.crop_roi(pcd, start=[-50, -50, self.vff_force], end=[50, 50, 0.2])
            pcd = self.voxel_down_sampling(pcd, voxel_size=self.voxel_size)
            pcd = self.rotate_point_cloud_by_pitch(pcd)

            # ship_body_bounds = {'min': [-1.1, -1, -1], 'max': [1.1, 1, 0.5]}
            ship_body_bounds = {'min': [-2, -2, -1], 'max': [2, 2, 0.5]}
            pcd = self.remove_ship_body(pcd, ship_body_bounds)

            points_cpu = pcd.point.positions.to(o3c.Device("CPU:0")).numpy()
            if points_cpu.shape[0] == 0:
                rospy.loginfo("Processed point cloud has no points.")
                return

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = msg_copy.header.frame_id
            points_xyz = pc2.create_cloud_xyz32(header, points_cpu)
            self.pub.publish(points_xyz)

            # print("Published processed pointcloud")
        except Exception as e:
            print("Error processing point cloud: ", e)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pointcloud_processor", anonymous=True)
    processor = PointCloudProcessor()
    processor.run()
