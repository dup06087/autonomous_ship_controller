#!/usr/bin/env python3


''' malloc memory problem detected'''
# don't use
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

def radius_outlier_removal(pcd):
    pcd, inliers = pcd.remove_radius_outlier(nb_points=5000, radius=1)

    # Numpy로 변환하고 inliers에 따라 데이터를 추출
    points = np.asarray(pcd.points)
    inlier_points = points[inliers]
    # outlier_points = np.delete(points, inliers, axis=0)

    # Inlier PointCloud
    inlier_cloud = o3d.geometry.PointCloud()
    inlier_cloud.points = o3d.utility.Vector3dVector(inlier_points)

    # Outlier PointCloud
    # outlier_cloud = o3d.geometry.PointCloud()
    # outlier_cloud.points = o3d.utility.Vector3dVector(outlier_points)

    # inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
    # outlier_cloud.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    # return outlier_cloud
    return inlier_cloud

def callback(pointcloud2_msg):
    pcd = pc2_to_o3d(pointcloud2_msg)
    outlier_removed = radius_outlier_removal(pcd)
    o3d.visualization.draw_geometries([outlier_removed])


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