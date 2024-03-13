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
import math
import socket
import select
import json
import threading
import subprocess

class LidarProcessing:
    def __init__(self):
        print('started')
        self.bbox_lists = []
        self.client_socket = None
        self.communication = threading.Thread(target= self.socket_send_obstacles)
        self.communication.start()

        # self.socket_send_obstacles()

        self.tracker = Tracker(max_lost=10)

        # 발행자 초기화
        self.pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=None, tcp_nodelay=True)
        self.marker_tracker_pub = rospy.Publisher('tracker_bounding_boxes', MarkerArray, queue_size=None, tcp_nodelay=True)

        rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback, queue_size=None, tcp_nodelay=True)
        print("spinning")
        rospy.spin()

    def kill_process_using_port(self, port_number):
        try:
            # lsof 명령을 실행하여 특정 포트를 사용 중인 프로세스 ID를 찾습니다.
            result = subprocess.check_output(f"lsof -i :{port_number} | grep LISTEN | awk '{{print $2}}'", shell=True).decode().strip()
            if result:
                process_id = result.split('\n')[0]  # 첫 번째 프로세스 ID를 가져옵니다.
                # kill 명령을 실행하여 프로세스를 종료합니다.
                subprocess.run(f"kill -9 {process_id}", shell=True)
                # print(f"Killed process {process_id} using port {port_number}")
            else:
                print(f"No process using port {port_number}")
        except Exception as e:
            print(f"Error: {e}")

    def socket_send_obstacles(self, client_socket='0.0.0.0', send_port=5000):
        self.kill_process_using_port(send_port)
        time.sleep(1)
        server_socket_pc_send_obstacle = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = client_socket
        port = send_port
        server_address = (host, port)
        server_socket_pc_send_obstacle.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket_pc_send_obstacle.bind(server_address)
        server_socket_pc_send_obstacle.listen(1)
        server_socket_pc_send_obstacle.settimeout(2)
        print("obstacle waiting client")
        
        print("connected client")
            # self.client_socket, client_address = server_socket_pc_send_obstacle.accept()
        

        # print("listening send")

        while True:
            try:
                self.client_socket, client_address = server_socket_pc_send_obstacle.accept()
                while True:
                    print("running obstacle")
                    time.sleep(1)
            except Exception as e:
                print("obstacle : ", e)




    def voxelization(self, pcd, voxel_size=0.05):
        downsampled = pcd.voxel_down_sample(voxel_size)
        return downsampled

    def crop_roi(self, pcd, start=[-10, -10, -5], end=[10, 10, 5]):
        min_bound = np.array(start)
        max_bound = np.array(end)
        roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        pcd = pcd.crop(roi_bounding_box)
        return pcd

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

    def bbox_coordinate(self, bounding_boxes):
        bbox_coordinates = []
        for bbox in bounding_boxes:
            center = bbox.get_center()
            bbox_coordinates.append(self.calculate_obstacle_coordinates(center[0], center[1]))
        print("bbox : ", bbox_coordinates)

    def marking(self, bounding_boxes_o3d):
        marker_array = MarkerArray()
        for i, bbox in enumerate(bounding_boxes_o3d):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.id = i
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
        self.pub.publish(marker_array)

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
            marker.pose.orientation.w = 1.0

            marker.scale.x = bb_width
            marker.scale.y = bb_height
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.a = 0.5

            marker_array.markers.append(marker)

        self.marker_tracker_pub.publish(marker_array)

    def pcd_3d_to_2d(self, pcd):
        pcd[:, 2] = 0
        pcd = np.unique(pcd, axis=0)
        return pcd

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

    def filter_by_intensity(self, pc_arr, threshold=30):
        mask = pc_arr['intensity'] > threshold
        return pc_arr[mask]

    def calculate_obstacle_coordinates(self, x, y, ship_lat=37.63129231587454, ship_lon=127.0760334199614, heading=0):
        heading_rad = math.radians(heading)
        delta_lat = y / 111111
        delta_lon = x / (111111 * math.cos(math.radians(ship_lat)))

        delta_x = delta_lon * math.cos(heading_rad) - delta_lat * math.sin(heading_rad)
        delta_y = delta_lon * math.sin(heading_rad) + delta_lat * math.cos(heading_rad)

        obstacle_lat = ship_lat + (delta_y * (180 / math.pi))
        obstacle_lon = ship_lon + (delta_x * (180 / math.pi) / math.cos(math.radians(ship_lat)))

        return obstacle_lat, obstacle_lon

    def do_tracker(self, bounding_boxes_o3d):
        bboxes = [(bbox.min_bound[0], bbox.min_bound[1], bbox.max_bound[0] - bbox.min_bound[0], bbox.max_bound[1] - bbox.min_bound[1]) for bbox in bounding_boxes_o3d]
        detection_scores = [1.0] * len(bboxes)
        class_ids = [0] * len(bboxes)
        tracks = self.tracker.update(bboxes, detection_scores, class_ids)
        return tracks
    
    def pc2_to_o3d(self, pc2_data):
        # print(pc2_data)
        pc_arr = pointcloud2_to_array(pc2_data)

        # intensity 기반 필터링
        filtered_pc_arr = self.filter_by_intensity(pc_arr, threshold=5)

        # 필터링된 데이터로 포인트 클라우드 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
        return pcd
    
    def callback(self, pointcloud2_msg):
        prev_time = time.time()
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
        self.bbox_lists = [bbox[2:6] for bbox in tracks]
        # print(self.bbox_lists)
        pc2_msg = self.o3d_to_pointcloud2(pcd)
        try:
            ready_to_read, ready_to_write, _ = select.select([], [self.client_socket], [], 1)
            print("ready to write? : ", ready_to_write)
            if ready_to_write:
                if isinstance(self.bbox_lists, list):
                    message = json.dumps(self.bbox_lists)
                    try:
                        self.client_socket.sendall(message.encode())
                        print("sent data : ", message.encode())
                    except OSError as e:
                        print("Error in sending message:", e)  # 占쏙옙占쏙옙 占쏙옙쨔占?占쌩곤옙
                        raise Exception("Connection with client has been closed.")
                else:
                    print("current_value is not a dictionary.")
        except Exception as e:
            print("obstacle sending error : " , e)
        self.pub.publish(pc2_msg)
        # print(time.time() - prev_time)

        # print("bboxes : ", self.bbox_lists)

        



def run_lidar():
    LidarProcessing()

if __name__ == "__main__":
    run_lidar()