#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math
import open3d as o3d
import numpy as np
import time
import numpy
import ros_numpy

# def callback(input_ros_msg):
#     pcds = pc2.read_points(input_ros_msg, skip_nans=True)
#     for data in pcds:
#         x2 = data[0]
#         y2 = data[1]
#         z2 = data[2]
#         intensity = data[3]
#         ring = data[4]
#         # time1 = datetime.fromtimestamp(data[5])
#         distance = math.sqrt((x2)**2 + (y2)**2 + (z2)**2)
#         azimuth = math.degrees(math.atan2((x2),(y2)))
#         # print(f"x : {x2}, y : {y2}, z : {z2}, intensity : {intensity}, ring : {ring}, distance : {distance}, azimuth : {azimuth}")
    
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pcds)
#     print(pcd.points)

# def callback(input_ros_msg):
#     # pass
#     pcd = pc2_to_o3d(input_ros_msg)
    
#     voxel_size = 0.05
#     downsampled = pcd.voxel_down_sample(voxel_size)

#     o3d.visualization.draw_geometries([downsampled])

# def pc2_to_o3d(pc2_data):
#     pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pc_arr)
#     return pcd

# if __name__ == "__main__":
#     pcd = None

#     rospy.init_node("listener", anonymous=True)
    
#     rospy.Subscriber("/velodyne_points", PointCloud2, callback)
#     rospy.spin()


def callback(pointcloud2_msg):
    pcd = pc2_to_o3d(pointcloud2_msg)

    voxel_size = 0.5
    downsampled_pcd = pcd.voxel_down_sample(voxel_size)

    header = pointcloud2_msg.header
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 0, PointField.FLOAT32, 1),
        PointField('z', 0, PointField.FLOAT32, 1)
    ]

    downsampled_points = np.asarray(downsampled_pcd.points)
    downsampled_pc2 = pc2.create_cloud(header, fields, downsampled_points)
    pub.publish(downsampled_pc2)

def pc2_to_o3d(pc2_data):
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_arr)
    return pcd

def listener():
    global pub
    rospy.init_node("pointcloud_listener", anonymous=True)
        
    pub = rospy.Publisher('/downsampled_points', PointCloud2, queue_size = 10)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback)
    rospy.spin()

if __name__ =="__main__":
    listener()