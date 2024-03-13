#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
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

class LidarProcessor:
    def __init__(self):
        self.tracker = Tracker(max_lost=10)
        self.bboxes = []
        self.bbox_lists = []
        self.flag_lidar = False
        self.lidar_processing_time = 0
        self.vff_force = []
        
        rospy.init_node("pointcloud_listener_and_publisher", anonymous=True) ### blocks if roslaunch not executed
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback, queue_size=None, tcp_nodelay=True)
        self.pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
        self.marker_tracker_pub = rospy.Publisher('tracker_bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
        

    def run(self):
        self.flag_lidar = True
        rospy.spin()
        self.flag_lidar = False

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
        filtered_pc_arr = self.filter_by_intensity(pc_arr, threshold=5)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
        return pcd

    def filter_by_intensity(self, pc_arr, threshold=30):
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

    def do_tracker(self, bounding_boxes_o3d):
        self.bboxes = [(bbox.min_bound[0], bbox.min_bound[1], bbox.max_bound[0] - bbox.min_bound[0], bbox.max_bound[1] - bbox.min_bound[1]) for bbox in bounding_boxes_o3d]
        detection_scores = [1.0] * len(self.bboxes)
        class_ids = [0] * len(self.bboxes)
        return self.tracker.update(self.bboxes, detection_scores, class_ids)



    def marking_tracks(self, tracks):
        marker_array = MarkerArray()
        for i, track in enumerate(tracks):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
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

    def callback(self, pointcloud2_msg):
        time_ = time.time()
        time_diff = rospy.Time.now() - pointcloud2_msg.header.stamp
        if time_diff.to_sec() > 0.05:
            return

        pcd = self.pc2_to_o3d(pointcloud2_msg)
        pcd = self.crop_roi(pcd, start=[-5, -2, -0.5], end=[5, 10, 0.5])
        pcd = self.voxelization(pcd, voxel_size=0.05)
        pcd, labels = self.DBSCAN(pcd, eps=0.1, min_points=5)
        bounding_boxes_o3d = self.compute_bounding_boxes(pcd, labels)

        tracks = self.do_tracker(bounding_boxes_o3d)
        self.marking_tracks(tracks)

        pc2_msg = self.o3d_to_pointcloud2(pcd)
        self.pub.publish(pc2_msg)

        self.bbox_lists = [bbox[2:6] for bbox in tracks]
        # print(self.bbox_lists)
        
        # self.calculate_vff_force(self.bbox_lists)
        
        self.lidar_processing_time = time.time() - time_
        # print("time processing : ", self.lidar_processing_time)
        
        
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