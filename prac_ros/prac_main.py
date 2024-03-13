#!/usr/bin/env python3

import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import ros_numpy
import std_msgs.msg
from matplotlib import pyplot as plt
import time

def voxelization(pcd, voxel_size = 0.05):
    downsampled = pcd.voxel_down_sample(voxel_size)
    return downsampled

def crop_roi(pcd, start = [-10, -10, -5], end = [10, 10, 5]):
    min_bound = np.array(start)  # 최소 x, y, z 값을 바꿔주세요
    max_bound = np.array(end)  # 최대 x, y, z 값을 바꿔주세요
    roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    pcd = pcd.crop(roi_bounding_box)
    return pcd

def compute_bounding_boxes(pcd, labels):
    max_label = labels.max()
    bounding_boxes = []
    for i in range(max_label + 1):
        point_indices = np.where(labels == i)[0]
        if len(point_indices) > 0:
            cluster = pcd.select_by_index(point_indices)
            bounding_box = cluster.get_axis_aligned_bounding_box()
            bounding_boxes.append(bounding_box)
    return bounding_boxes

def marking(bounding_boxes_o3d):
    marker_array = MarkerArray()

    for i, bbox in enumerate(bounding_boxes_o3d):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.id = i  # 각 마커에 고유 ID 부여
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        center = bbox.get_center()
        extent = bbox.get_extent()
        
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = extent[0]
        marker.scale.y = extent[1]
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def pcd_3d_to_2d(pcd):
    pcd[:, 2] = 0
    print(pcd)
    print(type(pcd))
    
    pcd = np.unique(pcd, axis=0)
    return pcd

def DBSCAN(pcd, eps, min_points):
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return pcd, labels

def o3d_to_pointcloud2(pcd_o3d):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'  # TODO: 해당 frame에 맞게 수정
    points_xyz = np.asarray(pcd_o3d.points)
    
    return pc2.create_cloud_xyz32(header, points_xyz)

def filter_by_intensity(pc_arr, threshold=30):
    # intensity 필드에서의 조건에 따라 데이터 필터링
    mask = pc_arr['intensity'] > threshold
    # 필터링된 데이터 반환
    return pc_arr[mask]

def callback(pointcloud2_msg):
    prev_time = time.time()

    time_diff = rospy.Time.now() - pointcloud2_msg.header.stamp
    # time_diff가 특정 시간 (예: 0.5초)보다 크면 메시지 무시
    if time_diff.to_sec() > 0.05:
        # print("Old message, ignoring... : {}".format(time_diff))
        return
    
    pcd = pc2_to_o3d(pointcloud2_msg)
    pcd = crop_roi(pcd, start = [-5, -2, -0.5], end = [5, 10, 0.5])
    
    pcd = voxelization(pcd, voxel_size=0.05)
    pcd, labels = DBSCAN(pcd, eps=0.1, min_points=5)
    bounding_boxes_o3d = compute_bounding_boxes(pcd, labels)

    marking(bounding_boxes_o3d)

    # 다시 rviz로 보내기 위해 PointCloud2로 변환
    pc2_msg = o3d_to_pointcloud2(pcd)
    

    pub.publish(pc2_msg)
    print(time.time() - prev_time)

def pc2_to_o3d(pc2_data):
    # print(pc2_data)
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_data)
    field_names = pc_arr.dtype.names
    print(field_names)

    # intensity 기반 필터링
    filtered_pc_arr = filter_by_intensity(pc_arr, threshold=5)

    # print(filtered_pc_arr)
    # 필터링된 데이터로 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
    return pcd

if __name__ == "__main__":
    print('started')

    prev_time = time.time()
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)
    print(time.time() - prev_time)
    prev_time = time.time()
    rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=None, tcp_nodelay=True)
    print(time.time() - prev_time)

    # 발행자 초기화
    prev_time = time.time()
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
    print(time.time() - prev_time)
    prev_time = time.time()
    marker_pub = rospy.Publisher('bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
    print(time.time()-prev_time)

    # 구독자 초기화
    prev_time = time.time()
    rospy.spin()
    print(time.time() - prev_time)