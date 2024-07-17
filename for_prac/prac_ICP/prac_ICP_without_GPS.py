import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Quaternion
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class ICPTest:
    def __init__(self):
        rospy.init_node('icp_test')
        self.prev_scan = None
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        
        # Initial position and heading
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_heading = 0.0  # Heading in degrees

        self.log_file = open("position_log.txt", "w")

    def lidar_callback(self, data):
        # Check if the timestamp is too old
        time_diff = rospy.Time.now() - data.header.stamp
        if time_diff.to_sec() > 0.05:  # Adjust this threshold as needed
            # rospy.logwarn("Dropping old point cloud data")
            return

        cloud = self.point_cloud2_to_o3d(data)
        cloud = self.downsample(cloud)
        if self.prev_scan is not None:
            # Perform ICP
            reg_p2p = o3d.pipelines.registration.registration_icp(
                cloud, self.prev_scan, 0.2,
                np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            transf = reg_p2p.transformation
            if reg_p2p.fitness > 0.5:  # Check fitness score
                translation = transf[:3, 3]
                rotation_matrix = transf[:3, :3]
                rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                # Update heading
                heading_change = np.degrees(rotation_euler[2])  # Yaw change in degrees
                self.current_heading += heading_change
                self.current_heading = self.current_heading % 360  # Normalize heading to [0, 360)

                # Calculate distance moved
                self.current_x += translation[0]
                self.current_y += translation[1]
                self.current_z += translation[2]

                # Print current position and heading
                rospy.loginfo(f"Current Position: x={self.current_x}, y={self.current_y}, z={self.current_z}")
                rospy.loginfo(f"Current Heading: {self.current_heading} degrees")

                # self.log_file.write(f"{rospy.Time.now().to_sec()}, {self.current_x}, {self.current_y}, {self.current_z}, {self.current_heading}\n")

                # Publish updated pose
                self.publish_pose(self.current_x, self.current_y, self.current_z, self.current_heading)
        self.prev_scan = cloud

    def point_cloud2_to_o3d(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points))
        return cloud

    def downsample(self, cloud, voxel_size=0.1):
        return cloud.voxel_down_sample(voxel_size)

    def rotation_matrix_to_euler(self, rotation_matrix):
        R = np.array(rotation_matrix)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def publish_pose(self, x, y, z, heading):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        # Heading to quaternion conversion
        quaternion = self.heading_to_quaternion(heading)
        pose_msg.pose.orientation = quaternion
        self.pose_pub.publish(pose_msg)

    def heading_to_quaternion(self, heading):
        rad = np.radians(heading)
        q = self.euler_to_quaternion(0, 0, rad)
        return Quaternion(*q)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def __del__(self):
        # Close the log file when the node is shut down
        self.log_file.close()

if __name__ == '__main__':
    try:
        ICPTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
