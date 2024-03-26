#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_array
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs.msg
from matplotlib import pyplot as plt
import time
from motrackers.tracker import Tracker
import math
import threading
from std_msgs.msg import ColorRGBA

class LidarProcessor:
    def __init__(self):
        self.pitch = None
        self.tracker = Tracker(max_lost=10)
        self.bboxes = []
        self.bbox_lists = []
        self.last_data_time = None
        self.flag_lidar = False
        self.lidar_processing_time = 0        
        self.cnt_no_pcd = 0
        
        self.vff_force = []
        self.init_coeff()
        # rospy.init_node("pointcloud_listener_and_publisher", anonymous=True) ### blocks if roslaunch not executed
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback, queue_size=None, tcp_nodelay=True)
        self.pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
        self.marker_tracker_pub = rospy.Publisher('tracker_bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
        self.marker_destination_pub = rospy.Publisher('destination_marker', Marker, queue_size=10)

        
        self.monitor_thread = threading.Thread(target=self.monitor_lidar_data)
        self.monitor_thread.start()
        
        self.changing_heading = 0

    def run(self):
        self.flag_lidar = True
        rospy.spin()
        self.flag_lidar = False
        


        
    def init_coeff(self):
        self.coeff_kf = 1
        self.coeff_kd = 1
        self.voxel_size = 0.05
        self.intensity = 5
        self.dbscan_eps= 0.1
        self.dbscan_minpoints = 5
        self.vff_force = 1
         
        
    def update_coeff(self, coeff_kf, coeff_kd, voxel_size, intensity, dbscan_eps, dbscan_minpoints, vff_force):
        self.coeff_kf = coeff_kf
        self.coeff_kd = coeff_kd
        self.voxel_size = voxel_size
        self.intensity = intensity
        self.dbscan_eps= dbscan_eps
        self.dbscan_minpoints = dbscan_minpoints
        self.vff_force = vff_force
        
    def voxelization(self, pcd, voxel_size=0.05):
        return pcd.voxel_down_sample(voxel_size)

    def crop_roi(self, pcd, start, end):
        min_bound = np.array(start)
        max_bound = np.array(end)
        roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        return pcd.crop(roi_bounding_box)

    def compute_bounding_boxes(self, pcd, labels):
        max_label = labels.max()
        bounding_boxes = []
        for i in range(max_label + 1):
            point_indices = np.where(labels == i)[0]
            if len(point_indices) > 0:
                cluster = pcd.select_by_index(point_indices)
                bounding_box = cluster.get_axis_aligned_bounding_box()
                bounding_boxes.append(bounding_box)
        return bounding_boxes


    def pc2_to_o3d(self, pc2_data):
        pc_arr = pointcloud2_to_array(pc2_data)
        filtered_pc_arr = self.filter_by_intensity(pc_arr, threshold=self.intensity)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
        return pcd

    def filter_by_intensity(self, pc_arr, threshold=5):
        mask = pc_arr['intensity'] > threshold
        return pc_arr[mask]

    def DBSCAN(self, pcd, eps, min_points):
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        return pcd, labels

    def o3d_to_pointcloud2(self, pcd_o3d):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'
        points_xyz = np.asarray(pcd_o3d.points)
        return pc2.create_cloud_xyz32(header, points_xyz)

    def do_tracker(self, bounding_boxes_o3d): # input : (top-left-x, top-left-y, width, height)
        self.bboxes = [(bbox.min_bound[0], bbox.min_bound[1], bbox.max_bound[0] - bbox.min_bound[0], bbox.max_bound[1] - bbox.min_bound[1]) for bbox in bounding_boxes_o3d]
        detection_scores = [1.0] * len(self.bboxes)
        class_ids = [0] * len(self.bboxes)
        return self.tracker.update(self.bboxes, detection_scores, class_ids) # return : (frame_id, track_id, bb_left, bb_top, bb_width, bb_height, conf, x, y, z).

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

    def rotate_point_cloud_by_pitch(self, pcd):
        """
        Rotate the point cloud by a given pitch angle.

        :param pcd: Open3D point cloud object to be rotated.
        :param pitch: The pitch angle in degrees to rotate the point cloud.
        :return: Rotated Open3D point cloud object.
        """
        
        # print('inner pitch : ', self.pitch)
        if self.pitch == None:
            return pcd
        
        # Convert pitch from degrees to radians
        # pitch_rad = np.radians(self.pitch)
        pitch_rad = np.radians(self.pitch)
        
        # Create the rotation matrix for pitch
        # R = np.array([[1, 0, 0],
        #               [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        #               [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
        
        
        R = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                      [0, 1, 0],
                      [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
        
        # Apply the rotation to each point in the point cloud
        pcd.rotate(R, center=(0, 0, 0))
        
        return pcd

    def delete_all_bbox_markers(self):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_tracker_pub.publish(marker_array)
        
    def marking_tracks(self, tracks):
        # self.delete_all_bbox_markers()
        
        marker_array = MarkerArray()
        for i, track in enumerate(tracks):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0)  # 마커가 지속되는 시간을 0초로 설정하여 즉시 사라지도록 설정
            bb_left, bb_top, bb_width, bb_height = track[2:6]
            marker.pose.position.x = bb_left + bb_width / 2
            marker.pose.position.y = bb_top + bb_height / 2
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = bb_width
            marker.scale.y = bb_height
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 0.5
            marker_array.markers.append(marker)
        self.marker_tracker_pub.publish(marker_array)

    def delete_destination_marker(self):
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self.marker_destination_pub.publish(delete_marker)
        
    def publish_destination_marker(self, x, y, z):
        self.delete_destination_marker()
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)  # 마커가 지속되는 시간을 0초로 설정하여 즉시 사라지도록 설정

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5  # 크기 조정
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # 초록색
        
        
        self.marker_destination_pub.publish(marker)
        
    def rotate_point_cloud_by_z(self, pcd):
        """
        Rotate the point cloud by 90 degrees around the Z-axis.

        :param pcd: Open3D point cloud object to be rotated.
        :return: Rotated Open3D point cloud object.
        """
        # Angle in radians (90 degrees)
        angle_rad = np.pi / 2
        
        # Rotation matrix for Z-axis rotation
        R = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                    [np.sin(angle_rad), np.cos(angle_rad), 0],
                    [0, 0, 1]])
        
        # Apply the rotation to each point in the point cloud
        pcd.rotate(R, center=(0, 0, 0))
        
        return pcd

    def monitor_lidar_data(self):
        while not rospy.is_shutdown():
            if self.last_data_time is not None:
                elapsed_time = time.time() - self.last_data_time
                if elapsed_time > 1:  # 5초 동안 데이터가 수신되지 않은 경우
                    rospy.logwarn("1초 동안 라이다 데이터를 수신하지 못했습니다.")
                    self.flag_lidar = False
                    # 필요한 추가 조치를 여기에 구현할 수 있습니다.
                else:
                    self.flag_lidar = True
            time.sleep(1)  # 1초에 한 번씩 체크

    def publish_empty_pointcloud2(self):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'
        
        # 포인트 필드 정의
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
        
        # 빈 포인트 클라우드 데이터
        points = []

        # 빈 포인트 클라우드 메시지 생성
        empty_pc2_msg = PointCloud2()
        empty_pc2_msg.header = header
        empty_pc2_msg.height = 1
        empty_pc2_msg.width = len(points)
        empty_pc2_msg.is_dense = False
        empty_pc2_msg.is_bigendian = False
        empty_pc2_msg.fields = fields
        empty_pc2_msg.point_step = 12  # FLOAT32 (4 bytes) * 3 (x, y, z)
        empty_pc2_msg.row_step = empty_pc2_msg.point_step * empty_pc2_msg.width
        empty_pc2_msg.data = []

        # 빈 포인트 클라우드 메시지 publish
        self.pub.publish(empty_pc2_msg)
    
    def callback(self, pointcloud2_msg):
        self.last_data_time = time.time()

        time_ = time.time()
        time_diff = rospy.Time.now() - pointcloud2_msg.header.stamp
        if time_diff.to_sec() > 0.05: # realtime
            return

        # if time_diff.to_sec() > 0.1: # rosbag
        #     return
        
        pcd = self.pc2_to_o3d(pointcloud2_msg)
        # pcd = self.rotate_point_cloud_by_z(pcd)

        # pcd = self.crop_roi(pcd, start=[-0.924, -0.36, -0.7], end=[1.5, 0.36, 0.2]) # axis - 정면 x, 왼쪽 y, 위 z # little bit front for check 
        # pcd = self.crop_roi(pcd, start=[-0.924, -0.36, -0.7], end=[0.96, 0.36, 0.2]) # axis - 정면 x, 왼쪽 y, 위 z # no obstacle, little bigger than crop roi 
        # pcd = self.crop_roi(pcd, start=[-1, -5, -0.5], end=[10, 5, 0.5]) # axis - 정면 x, 왼쪽 y, 위 z # original
        pcd = self.crop_roi(pcd, start=[-1, -5, -0.5], end=[20, 5, 0.5]) # axis - 정면 x, 왼쪽 y, 위 z # original
        # pcd = self.crop_roi(pcd, start=[-20, -20, -5], end=[20, 20, 5]) # axis - 정면 x, 왼쪽 y, 위 z # test
        
        pcd = self.rotate_point_cloud_by_pitch(pcd)  # 여기에서 포인트 클라우드 회전 적용
        ship_body_bounds = {'min': [-0.925, -0.35, -0.6], 'max': [0.95, 0.35, 0.1]}  # 선체가 위치하는 영역을 지정
        pcd = self.remove_ship_body(pcd, ship_body_bounds)
       
        pcd = self.voxelization(pcd, voxel_size=self.voxel_size)
        
        # print("bbox lists ", self.bbox_lists)
        if len(pcd.points) == 0:  # 포인트 클라우드 내 포인트의 수가 0인지 확인
            self.cnt_no_pcd += 1
            if self.cnt_no_pcd >= 5:
                self.bbox_lists = []
                self.publish_empty_pointcloud2()  # 수정된 부분
                self.delete_all_bbox_markers()
                print("init bbox : no pcd found, count : ", self.cnt_no_pcd)
                        

            return
        
        self.cnt_no_pcd = 0

        pcd, labels = self.DBSCAN(pcd, eps=self.dbscan_eps, min_points=int(self.dbscan_minpoints))
        
        bounding_boxes_o3d = self.compute_bounding_boxes(pcd, labels)

        tracks = self.do_tracker(bounding_boxes_o3d)
        self.marking_tracks(tracks)

        pc2_msg = self.o3d_to_pointcloud2(pcd)
        self.pub.publish(pc2_msg)

        self.bbox_lists = [bbox[2:6] for bbox in tracks]
        # print("bbox lists ", self.bbox_lists)
        # self.calculate_vff_force(self.bbox_lists)
        
        self.lidar_processing_time = time.time() - time_
        if self.lidar_processing_time >= 0.25:
            print("time processing : ", self.lidar_processing_time)
        
        
    def calculate_vff_force(self, obstacles):
        """
        Calculate the VFF force for a given list of obstacles.
        
        :param obstacles: List of obstacles where each obstacle is represented as [x, y, w, h]
        :return: The resulting VFF force as a numpy array
        """
        ship_position = np.array([0, 0])  # 선박의 현재 위치
        ship_direction = np.array([1, 0])  # 선박의 현재 방향

        # 장애물에서의 반발력을 계산
        repulsive_forces = []
        for obs in obstacles:
            center = np.array([obs[0], obs[1]])  # 장애물 중심
            distance = np.linalg.norm(center - ship_position)  # 선박과 장애물 사이 거리(스칼라)
            direction = (center - ship_position) / distance  # 선박에서 장애물로의 단위 벡터
            force_magnitude = 1 / distance**2  # 반발력의 크기 (거리의 제곱에 반비례)
            repulsive_forces.append(-force_magnitude * direction)  # 반발력 (반대 방향)

        # 모든 장애물에 대한 반발력의 합
        self.vff_force = np.sum(repulsive_forces, axis=0).tolist()
        # print("output force : ", self.vff_force)


if __name__ == "__main__":

    lidar_processor = LidarProcessor()
    lidar_processor_thread = threading.Thread(target = lidar_processor.run)
    lidar_processor_thread.start()
    
    while True:
        time.sleep(1)
        print("lidar status : ", lidar_processor.flag_lidar)