#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_array
import std_msgs.msg
from matplotlib import pyplot as plt
import time
from motrackers.tracker import Tracker

global bboxes, bbox_lists
bboxes = []
bbox_lists = []

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

def bbox_coordinate(bounding_boxes):
    bbox_coordinates = []
    for bbox in bounding_boxes:
        center = bbox.get_center()
        bbox_coordinates.append(calculate_obstacle_coordinates(center[0], center[1]))
    print("bbox : ", bbox_coordinates)

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

        marker.color.b = 1.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)

def marking_tracks(tracks):
    marker_array = MarkerArray()

    for i, track in enumerate(tracks):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.id = i  # 각 마커에 track_id를 고유 ID로 부여
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Track 정보에서 bounding box의 좌상단 좌표와 너비, 높이를 가져옵니다.
        bb_left, bb_top, bb_width, bb_height = track[2:6]
        
        # bounding box의 중심 좌표를 계산합니다.
        marker.pose.position.x = bb_left + bb_width / 2
        marker.pose.position.y = bb_top + bb_height / 2
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        # bounding box의 크기를 설정합니다.
        marker.scale.x = bb_width
        marker.scale.y = bb_height
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

    marker_tracker_pub.publish(marker_array)

def pcd_3d_to_2d(pcd):
    pcd[:, 2] = 0
    pcd = np.unique(pcd, axis=0)
    return pcd

def DBSCAN(pcd, eps, min_points):
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    # print(labels)
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

import math

def calculate_obstacle_coordinates(x, y, ship_lat = 37.63129231587454 , ship_lon = 127.0760334199614, heading = 0):
    # Convert heading to radians
    heading_rad = math.radians(heading)

    # Convert x and y distances from meters to degrees
    delta_lat = y / 111111  # 1 degree is approximately 111,111 meters
    delta_lon = x / (111111 * math.cos(math.radians(ship_lat)))

    # Rotate x and y distances based on heading
    delta_x = delta_lon * math.cos(heading_rad) - delta_lat * math.sin(heading_rad)
    delta_y = delta_lon * math.sin(heading_rad) + delta_lat * math.cos(heading_rad)

    # Calculate the new coordinates
    obstacle_lat = ship_lat + (delta_y * (180 / math.pi))
    obstacle_lon = ship_lon + (delta_x * (180 / math.pi) / math.cos(math.radians(ship_lat)))

    return obstacle_lat, obstacle_lon
'''
# Example usage
ship_lat = 37.4219999
ship_lon = -122.0840575
heading = 45  # degrees
x = 35.5  # meters
y = 20.0  # meters

obstacle_lat, obstacle_lon = calculate_obstacle_coordinates(ship_lat, ship_lon, heading, x, y)
print(f"Obstacle coordinates: Latitude {obstacle_lat}, Longitude {obstacle_lon}")
'''
def do_tracker(bounding_boxes_o3d):
    global bboxes
    bboxes = [(bbox.min_bound[0], bbox.min_bound[1], bbox.max_bound[0] - bbox.min_bound[0], bbox.max_bound[1] - bbox.min_bound[1]) for bbox in bounding_boxes_o3d]

    # print("bounding boxes : ", bboxes)
    detection_scores = [1.0] * len(bboxes)  # Assuming a detection score of 1.0 for all bounding boxes
    class_ids = [0] * len(bboxes)  # Assuming a class ID of 0 for all bounding boxes
    # print("bbox : ", bboxes[0])
    # print("detectino score " ,detection_scores)
    # Update the tracker
    tracks = tracker.update(bboxes, detection_scores, class_ids)
    # print("tracks : ", tracks)
    return tracks

def callback(pointcloud2_msg):
    global tracker, bbox_lists
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

    # marking(bounding_boxes_o3d)
    # bbox_coordinate(bounding_boxes_o3d)

    tracks = do_tracker(bounding_boxes_o3d)
    marking_tracks(tracks)

    # 다시 rviz로 보내기 위해 PointCloud2로 변환
    pc2_msg = o3d_to_pointcloud2(pcd)
    
    pub.publish(pc2_msg)
    print(time.time() - prev_time)

    bbox_lists = [bbox[2:6] for bbox in tracks]

    return bbox_lists

def pc2_to_o3d(pc2_data):
    # print(pc2_data)
    pc_arr = pointcloud2_to_array(pc2_data)

    # intensity 기반 필터링
    filtered_pc_arr = filter_by_intensity(pc_arr, threshold=5)

    # 필터링된 데이터로 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
    return pcd


def run_lidar():
    global tracker, pub, marker_tracker_pub, bbox_lists, bbox
    print('started')
    tracker = Tracker(max_lost=10)

    prev_time = time.time()
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=None, tcp_nodelay=True)

    # 발행자 초기화
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
    # marker_pub = rospy.Publisher('bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
    marker_tracker_pub = rospy.Publisher('tracker_bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
    # 구독자 초기화
    print(time.time() - prev_time)
    rospy.spin()

if __name__ == "__main__":
    run_lidar()
    # print('started')
    # tracker = Tracker(max_lost=10)

    # prev_time = time.time()
    # rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)
    # rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=None, tcp_nodelay=True)

    # # 발행자 초기화
    # pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
    # # marker_pub = rospy.Publisher('bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
    # marker_tracker_pub = rospy.Publisher('tracker_bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)
    # # 구독자 초기화
    # print(time.time() - prev_time)
    # rospy.spin()
    