#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import ros_numpy
import std_msgs.msg
from matplotlib import pyplot as plt
import time
import struct

def o3d_to_pointcloud2(pcd_o3d):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1)]

    points = []
    points_o3d = np.asarray(pcd_o3d.points)
    colors_o3d = np.asarray(pcd_o3d.colors)
    
    for i in range(len(points_o3d)):
        point = points_o3d[i]
        color = colors_o3d[i]
        
        r = int(color[0] * 255.0)
        g = int(color[1] * 255.0)
        b = int(color[2] * 255.0)
        a = 255
        
        rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = list(point) + [rgba]
        points.append(pt)
    
    return pc2.create_cloud(header, fields, points)

def DBSCAN(pcd, eps, min_points):
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return pcd, labels

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
        marker.color.a = 0.8

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def callback(pointcloud2_msg):
    prev_time = time.time()
    pcd = pc2_to_o3d(pointcloud2_msg)
    
    pcd, labels = DBSCAN(pcd, eps=0.2, min_points=5)
    bounding_boxes_o3d = compute_bounding_boxes(pcd, labels)

    marking(bounding_boxes_o3d)

    pc2_msg = o3d_to_pointcloud2(pcd)
    print(f"time : {time.time()-prev_time}")
    pub.publish(pc2_msg)

def pc2_to_o3d(pc2_data):
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_arr)
    return pcd

if __name__ == "__main__":
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)

    # Publishers initialization
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=2)
    marker_pub = rospy.Publisher('bounding_boxes', MarkerArray, queue_size=2)

    # Subscriber initialization
    rospy.Subscriber("/velodyne_points", PointCloud2, callback)

    rospy.spin()
