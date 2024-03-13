#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import ros_numpy
import std_msgs.msg
import time

def filter_by_intensity(pc_arr, threshold=30):
    # intensity 필드에서의 조건에 따라 데이터 필터링
    mask = pc_arr['intensity'] > threshold
    # 필터링된 데이터 반환
    return pc_arr[mask]

def callback(pointcloud2_msg):
    global cnt
    time_diff = rospy.Time.now() - pointcloud2_msg.header.stamp
    # time_diff가 특정 시간 (예: 0.5초)보다 크면 메시지 무시
    if time_diff.to_sec() > 0.05:
        # print("Old message, ignoring... : {}".format(time_diff))
        return

    prev_time = time.time()
    pcd = pc2_to_o3d(pointcloud2_msg)
    
    '''data processing part'''

    # 다시 rviz로 보내기 위해 PointCloud2로 변환
    pc2_msg = o3d_to_pointcloud2(pcd)
    pub.publish(pc2_msg)
    # print(time.time() - prev_time)


def o3d_to_pointcloud2(pcd_o3d):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()

    header.frame_id = 'velodyne'  # TODO: 해당 frame에 맞게 수정
    points_xyz = np.asarray(pcd_o3d.points)
    return pc2.create_cloud_xyz32(header, points_xyz)

def pc2_to_o3d(pc2_data):
    # print(pc2_data)
    pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_data)
    field_names = pc_arr.dtype.names
    print(field_names)

    # intensity 기반 필터링
    filtered_pc_arr = filter_by_intensity(pc_arr)

    # print(filtered_pc_arr)
    # 필터링된 데이터로 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.column_stack([filtered_pc_arr['x'], filtered_pc_arr['y'], filtered_pc_arr['z']]))
    return pcd

if __name__ == "__main__":
    print('started')
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True)

    # 발행자 초기화
    pub = rospy.Publisher('/o3d_pointcloud', PointCloud2, queue_size=1)

    # 구독자 초기화
    rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=1)

    rospy.spin()