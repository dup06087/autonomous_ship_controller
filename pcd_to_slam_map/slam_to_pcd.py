import numpy as np
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
from sensor_msgs import point_cloud2

def o3d_to_ros(pcd_o3d, frame_id="map"):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    points = np.asarray(pcd_o3d.points)
    
    # 포인트 클라우드에 RGB 정보가 있는 경우 이 부분을 수정해야 합니다.
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    
    return point_cloud2.create_cloud(header, fields, points)

def publish_ply_as_pointcloud2(ply_file_path, topic_name):
    rospy.init_node('ply_publisher_node', anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    pcd = o3d.io.read_point_cloud(ply_file_path)  # PLY 파일 로드
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        ros_pcd = o3d_to_ros(pcd)  # Open3D 포인트 클라우드를 ROS 메시지로 변환
        pub.publish(ros_pcd)
        rate.sleep()

if __name__ == '__main__':
    ply_file_path = 'combined_pcd.ply'
    topic_name = 'your_ros_topic_name'
    publish_ply_as_pointcloud2(ply_file_path, topic_name)