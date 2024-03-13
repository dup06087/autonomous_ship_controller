#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import ros_numpy
import std_msgs.msg

def o3d_to_pointcloud2(pcd_o3d):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'  # TODO: 해당 frame에 맞게 수정
    points_xyz = np.asarray(pcd_o3d.points)
    return pc2.create_cloud_xyz32(header, points_xyz)

def crop_roi(pcd):
    min_bound = np.array([-10, -10, -20])  # 최소 x, y, z 값을 바꿔주세요
    max_bound = np.array([10, 10, 20])  # 최대 x, y, z 값을 바꿔주세요
    roi_bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    pcd = pcd.crop(roi_bounding_box)
    return pcd

def callback(pointcloud2_msg):
    pcd = pc2_to_o3d(pointcloud2_msg)
    pcd = crop_roi(pcd)

    # 다시 rviz로 보내기 위해 PointCloud2로 변환
    pc2_msg = o3d_to_pointcloud2(pcd)
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
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=1, tcp_nodelay=True)

    # 구독자 초기화
    rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=1,tcp_nodelay=True)

    rospy.spin()
