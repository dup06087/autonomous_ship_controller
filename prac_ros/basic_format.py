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

def callback(pointcloud2_msg):
    pcd = pc2_to_o3d(pointcloud2_msg)
    o3d.visualization.draw_geometries([pcd])


def pc2_to_o3d(pc2_data):
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_arr)
    return pcd

def listener():
    rospy.init_node("pointcloud_listener", anonymous=True)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback)
    rospy.spin()

if __name__ =="__main__":
    listener()