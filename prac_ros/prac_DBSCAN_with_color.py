#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
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


def DBSCAN(pcd, eps = 0.05, min_points=10):
        
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.05, min_points=10, print_progress=True))

    print(labels)
    print(type(labels))
    max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    print(type(pcd))
    print(pcd)
    return pcd

def callback(pointcloud2_msg):
    prev_time = time.time()
    pcd = pc2_to_o3d(pointcloud2_msg)
    
    pcd = DBSCAN(pcd, 0.5, 20)

    # 다시 rviz로 보내기 위해 PointCloud2로 변환
    pc2_msg = o3d_to_pointcloud2(pcd)
    print(f"time : {time.time()-prev_time}")
    pub.publish(pc2_msg)

def pc2_to_o3d(pc2_data):
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_arr)
    return pcd

if __name__ == "__main__":
    print('started')
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)

    # 발행자 초기화
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=10)
    marker_pub = rospy.Publisher('bounding_boxes', Marker, queue_size=10)

    # 구독자 초기화
    rospy.Subscriber("/velodyne_points", PointCloud2, callback)

    rospy.spin()
