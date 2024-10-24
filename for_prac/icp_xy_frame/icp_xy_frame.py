import rospy
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import open3d.core as o3c
import os
import numpy as np
import math
import time
import json
import datetime
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import sys
import signal

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 수동으로 입력한 원래 값 (도와 분 형식)


'''lane_direction'''
# initial_latitude_raw = 3737.7302554
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7049263
# initial_longitude_direction = 'E'
# initial_heading =8.133

'''lane_vertial_direction'''
# initial_latitude_raw = 3737.7337296
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7078467
# initial_longitude_direction = 'E'
# initial_heading =282.903


'''rect1'''
initial_latitude_raw = 3737.7326613
initial_latitude_direction = 'N'
initial_longitude_raw = 12704.7064656
initial_longitude_direction = 'E'
initial_heading = 8.955

'''rhombus'''
# initial_latitude_raw = 3737.7328295
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7068961
# initial_longitude_direction = 'E'
# initial_heading = 335.797

'''round1'''
# initial_latitude_raw = 3737.7327122
# initial_latitude_direction = 'N'
# initial_longitude_raw = 12704.7064321
# initial_longitude_direction = 'E'
# initial_heading =9.532


# CUDA 장치 설정
device = o3c.Device("CUDA:0")  # CUDA 장치 0을 사용

def warm_up_open3d_cuda():
    print("Warming up CUDA using Open3D-Core...")

    # 간단한 텐서를 생성하고 CUDA 디바이스에 올림
    tensor_a = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)
    tensor_b = o3c.Tensor(np.random.rand(100).astype(np.float32), device=device)

    # 두 텐서를 더하는 간단한 연산을 수행하여 CUDA 연산을 실행
    tensor_c = tensor_a + tensor_b

    # 결과 확인을 위해 CPU로 다시 복사
    result = tensor_c.cpu().numpy()
    
    print("CUDA warm-up complete.")
    return result

class IMUCorrector:
    def __init__(self):
        print("IMUCorrector Initialized")
        self.correction_pub = rospy.Publisher('/imu/corrected', Float64MultiArray, queue_size=10)
        self.raw_imu_pub = rospy.Publisher('/example/imu', Imu, queue_size=10)  # Republish to /example/imu

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # IMU deltas (relative to last ICP result)
        self.dnorth = 0  # X position delta (local)
        self.deast = 0  # Y position delta (local)
        self.dyaw = 0  # Heading delta
        self.dyaw_step = 0
        
        # Velocity initialized to zero
        self.vx = 0  # Velocity in x-direction
        self.vy = 0  # Velocity in y-direction
        self.prev_heading = initial_heading
        
        self.prev_time = None

    def imu_callback(self, data):
        # print('imu : ', rospy.Time.now())

        """Accumulate IMU changes for the next ICP step."""
        current_time = rospy.Time.now()
        
        time_diff = (current_time - data.header.stamp).to_sec()

        if time_diff > 0.1:
            return
        
        data.header.frame_id = "odom"  # Set the appropriate frame here

        # Republish the data to /example/imu
        self.raw_imu_pub.publish(data)
        
        # If prev_time is None (i.e., first callback), initialize it
        if self.prev_time is None:
            self.prev_time = current_time
            print("Initializing prev_time:", self.prev_time)
            return  # Skip the first calculation since we can't compute delta_time yet

        # Calculate delta time between IMU callbacks
        delta_time = (current_time - self.prev_time).to_sec()

        # Extract angular velocity for heading (z-axis rotation, i.e., yaw rate)
        angular_velocity_z = data.angular_velocity.z  # Angular velocity around z-axis (yaw rate)

        # Update heading using angular velocity (change in heading = angular velocity * time)
        self.dyaw_step = angular_velocity_z * delta_time * 180 / math.pi
        self.dyaw += self.dyaw_step 
        
        
        # Update the previous time for the next callback
        self.prev_time = current_time

    def remove_gravity_and_correct(self, ax, ay, az, roll, pitch, yaw):
        # Yaw를 0으로 만들기 위한 회전 행렬 계산
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        yaw_correction = R.from_euler('z', -yaw)
        corrected_rotation = yaw_correction * rotation

        # 중력 벡터 계산 (z축 방향으로 9.81 m/s²)
        g = 9.81
        gravity = np.array([0, 0, g])

        # Yaw를 상쇄한 회전 행렬로 중력 벡터 변환
        rotation_matrix = corrected_rotation.as_matrix()
        gravity_local = rotation_matrix.T @ gravity

        # 가속도에서 중력 성분 제거 (free acceleration 계산)
        free_acc_x = ax - gravity_local[0]
        free_acc_y = ay - gravity_local[1]
        free_acc_z = az - gravity_local[2]

        return free_acc_x, free_acc_y, free_acc_z
    
    def pre_icp_reset(self):
        """Reset only the IMU deltas before ICP starts, keep velocities intact."""
        self.dyaw = 0
        # print("IMU deltas reset before ICP, velocities intact.")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def rotate_acceleration(self, ax, ay, az, roll, pitch, yaw):
        """Rotate acceleration data from IMU's local frame to global frame."""
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        R = np.dot(R_z, np.dot(R_y, R_x))
        acceleration = np.array([ax, ay, az])
        corrected_acceleration = np.dot(R, acceleration)
        
        return corrected_acceleration


class ICPHandler:
    def __init__(self, imu_corrector, experiment_folder):
        self.imu_corrector = imu_corrector
        self.experiment_folder = experiment_folder
        self.icp_data_file = os.path.join(self.experiment_folder, "icp_log.txt")
        self.prev_scan = None

        # ICP global position (latitude, longitude, heading)
        self.prev_x_global = 0
        self.prev_y_global = 0
        self.prev_yaw = 0

        # ICP initial guess
        self.icp_initial_guess = np.eye(4)
        self.icp_result_pub = rospy.Publisher('/icp_result', Float64MultiArray, queue_size=1)
        self.odom_pub = rospy.Publisher('/example/odom', Odometry, queue_size=10)  # NEW ODOM PUBLISHER
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback, queue_size=1)

        with open(self.icp_data_file, 'w') as f:
            f.write("")
            
        self.processed_time = None

        self.dx = 0
        self.dy = 0
        self.dyaw = 0

        self.prev_x_moved = 0
        self.prev_y_moved = 0
        self.prev_yaw_changed = 0
        
    def lidar_callback(self, data):
        # print('lidar : ', rospy.Time.now())
        try:
            # Get the current time and check how much time has passed since the last scan
            current_time = rospy.Time.now()
            time_diff = (current_time - data.header.stamp).to_sec()
            
            if time_diff > 0.1:
                # print("time diff : ", time_diff)
                return
            if self.processed_time == None:
                self.processed_time = current_time
                return

            self.dyaw = self.imu_corrector.dyaw

            # Reset IMU deltas at the start of the callback
            self.imu_corrector.pre_icp_reset()

            # Convert PointCloud2 message to Open3D format
            cloud = self.point_cloud2_to_o3d(data)

            cloud = self.downsample(cloud)

            # cloud = self.translate_pointcloud(cloud, distance=0.5)  # 0.5m를 예시로 적용


            if self.prev_scan is not None:
                # Use IMU's deltas as the initial guess for ICP
                self.apply_imu_to_icp_guess()

                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud,
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(
                        relative_fitness=1e-6,
                        relative_rmse=1e-6,
                        max_iteration=5
                    ),
                )

                # If ICP fitness is good, update position and heading
                if reg_gicp.fitness > 0.8:
                    transf = reg_gicp.transformation.cpu().numpy()
                    translation = transf[:3, 3]
                    x_moved_local = translation[0]
                    y_moved_local = translation[1]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    # Update heading based on ICP result
                    icp_yaw_change = np.degrees(rotation_euler[2])
                    current_yaw = (self.prev_yaw + icp_yaw_change) % 360

                    # self.calculate_matrix(transf)
                    time_diff = (data.header.stamp - self.processed_time).to_sec()
                    #Calculate new global position based on ICP translation result
                    x_global, y_global, v_x_global, v_y_global, x_moved_global, y_moved_global = self.calculate_new_position(
                        self.prev_x_global, self.prev_y_global,
                        x_moved_local, y_moved_local, self.prev_yaw, time_diff
                    )
                    
                    # Update global position with ICP result
                    self.prev_x_global = x_global
                    self.prev_y_global = y_global
                    self.prev_yaw = current_yaw
                    # self.imu_corrector.post_icp_reset(v_x, v_y)
                    print("calculated velocity : ", v_x_global, v_y_global)
                    self.prev_x_moved = x_moved_local
                    self.prev_y_moved = y_moved_local
                    self.prev_yaw_changed = icp_yaw_change / time_diff
                    
                    
                    robot_frame_velocity_x = translation[0] / time_diff
                    robot_frame_velocity_y = translation[1] / time_diff
                    # Publish updated ICP result
                    # self.publish_icp_result(x, y, current_yaw, translation[0], translation[1], v_x, v_y, icp_yaw_change)
                    self.publish_icp_result(x_global, y_global, current_yaw, translation[0], translation[1], robot_frame_velocity_x, robot_frame_velocity_y, icp_yaw_change)


                    # Log the result to file
                    self.log_icp_result_to_file(x_global, y_global, current_yaw, v_x_global, v_y_global, x_moved_local, y_moved_local, x_moved_global, y_moved_global, time_diff, data.header.stamp)
                    self.processed_time = current_time
                    
                else:
                    print("low accuracy")
                    # return 
            else:
                print("prev_scan none : ", rospy.Time.now())
            # Store the current scan for the next iteration
            self.prev_scan = cloud

        except Exception as e:
            print(f"ICP error: {e}")
        
    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading, delta_time):
        """Convert local ICP dx, dy to latitude and longitude updates, and calculate velocities."""
        if heading > 180:
            heading -=360
        heading_rad = math.radians(heading)
        
        # Convert local dx, dy to global north/east deltas
        delta_x_global = delta_x * math.cos(heading_rad) - (delta_y) * math.sin(heading_rad)
        delta_y_global = delta_x * math.sin(heading_rad) + (delta_y) * math.cos(heading_rad)

        lat += delta_x_global
        lon += delta_y_global
        
        # Calculate velocities based on the position deltas and time difference
        v_x_global = delta_x_global / delta_time
        v_y_global = delta_y_global / delta_time

        print(f"New Lat: {lat}, New Lon: {lon}, v_x: {v_x_global}, v_y: {v_y_global}, delta_time: {delta_time}")
        return lat, lon, v_x_global, v_y_global, delta_x_global, delta_y_global
    
    def apply_imu_to_icp_guess(self):
        """Use the IMU-based deltas to set the initial guess for ICP."""
        # dx dy는 dnorth deast기준이므로 이전 위치 heading으로 해야함. 현재X 
        # heading_rad = np.radians(self.prev_heading) 
    
        # # Compute local frame deltas using the inverse rotation matrix
        # dx = self.dnorth * math.cos(heading_rad) + self.deast * math.sin(heading_rad)
        # dy = self.dnorth * math.sin(heading_rad) - self.deast * math.cos(heading_rad)

        #이거는 dheading()
        heading_diff = np.radians(self.dyaw)
        # heading_diff = np.radians(self.prev_heading_changed)
        # print("dheading : ", self.dheading)
        # print("dx dy : ", dx, dy)
        # print("prev x y : ", self.prev_x_moved, self.prev_y_moved)
        self.icp_initial_guess = np.array([
            [np.cos(heading_diff), -np.sin(heading_diff), 0, self.prev_x_moved],
            [np.sin(heading_diff), np.cos(heading_diff),  0, self.prev_y_moved],
            [0,             0,              1, 0],
            [0,             0,              0, 1]
        ])
        
        # self.icp_initial_guess = np.array([
        #     [np.cos(heading_diff), -np.sin(heading_diff), 0, dx],
        #     [np.sin(heading_diff), np.cos(heading_diff),  0, dy],
        #     [0,             0,              1, 0],
        #     [0,             0,              0, 1]
        # ])
        
        # self.icp_initial_guess = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1,  0, 0],
        #     [0,             0,              1, 0],
        #     [0,             0,              0, 1]
        # ])
        
        # print("self.icp_initial_guess : ", self.icp_initial_guess)

    def publish_icp_result(self, lat, lon, heading):
        msg = Float64MultiArray()
        msg.data = [lat, lon, heading]
        self.icp_result_pub.publish(msg)

    # Modify this method to also publish odometry
    def publish_icp_result(self, lat, lon, heading, dx, dy, v_x, v_y, icp_heading_change):
        # Publish the original ICP result as Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [lat, lon, heading]
        self.icp_result_pub.publish(msg)

        # Now also publish odometry
        self.publish_odometry(lat, lon, heading, v_x, v_y, icp_heading_change)

    def publish_odometry(self, lat, lon, heading, v_x, v_y, angular_velocity_z):
        # Create and populate the Odometry message
        odom_msg = Odometry()

        # Fill out the header
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # You don't need to provide position data here because you are focusing on velocity and heading change.
        odom_msg.pose.pose.position.x = lat
        odom_msg.pose.pose.position.y = lon
        odom_msg.pose.pose.position.z = 0  # Assuming 2D plane

        heading_rad = math.radians(heading)
        # heading을 쿼터니언으로 변환
        rotation = R.from_euler('z', heading_rad)  # z축을 기준으로 회전
        quaternion = rotation.as_quat()  # [x, y, z, w]
        print("quaternion : ", quaternion)
        # Orientation can stay zero as you are only concerned about the delta heading (angular velocity)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]  # No rotation

        # Set the velocity (linear and angular)
        odom_msg.twist.twist.linear.x = v_x  # Velocity in x-direction
        # odom_msg.twist.twist.linear.x = 0  # Velocity in x-direction
        odom_msg.twist.twist.linear.y = v_y  # Velocity in y-direction
        # odom_msg.twist.twist.linear.y = 0  # Velocity in y-direction
        odom_msg.twist.twist.linear.z = 0

        # Set angular velocity as -1 * icp_heading_change for delta heading (yaw rate)
        odom_msg.twist.twist.angular.z = angular_velocity_z * math.pi / 180
        # odom_msg.twist.twist.angular.z = 0

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)
        
    def log_icp_result_to_file(self, lat, lon, heading, v_x, v_y, x_moved_local, y_moved_local, x_moved_global, y_moved_global, time_diff, stamp):
        """Logs the ICP result to a file in JSON format with time, dx, dy, dheading."""
        # Convert ROS time (stamp) to the required format (HHMMSS.milis)
        sec = stamp.secs  # Seconds since epoch
        nsec = stamp.nsecs  # Nanoseconds part

        # Convert seconds to datetime
        rostime = datetime.datetime.fromtimestamp(sec) - datetime.timedelta(hours=9)
        # Format the time as HHMMSS.milis
        formatted_time = rostime.strftime("%H%M%S") + f".{int(nsec / 1e6):03d}"

        # Prepare the log entry with additional dx, dy, dheading
        log_entry = {
            'x': lat,
            'y': lon,
            'yaw': heading,
            'v_x' : v_x,
            'v_y' : v_y,
            'x_moved_local' : x_moved_local,
            'y_moved_local' : y_moved_local,
            'x_moved_global' : x_moved_global,
            'y_moved_global' : y_moved_global,
            'time_diff' : time_diff,
            'time': formatted_time,  # Add the time key in the requested format
        }

        # Write to log file in JSON format
        with open(self.icp_data_file, 'a') as f:
            f.write(json.dumps(log_entry) + '\n')
        print(f"Logged ICP result: {log_entry}")

    def point_cloud2_to_o3d(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(np.array(points), dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return cloud

    def downsample(self, cloud, voxel_size=0.1):
        return cloud.voxel_down_sample(voxel_size)

    def rotation_matrix_to_euler(self, rotation_matrix):
        """Convert a rotation matrix to Euler angles."""
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
    
    def translate_pointcloud(self, cloud, distance):
        """
        PointCloud를 주어진 거리만큼 차량의 heading 방향으로 평행 이동하는 함수.
        
        :param cloud: Open3D PointCloud 객체
        :param distance: 이동시킬 거리 (미터 단위, GNSS와 LiDAR 간의 거리)
        :param heading: 차량의 heading 값 (라디안 단위)
        :return: 평행 이동된 PointCloud
        """
        # heading 각도에 맞춰 이동할 x, y 성분 계산        
        # PointCloud의 모든 점들을 dx, dy 만큼 이동시킴
        

        cloud = cloud.translate((0, distance, 0))

        return cloud

if __name__ == "__main__":
    rospy.init_node("imu_icp_fusion_node")
    warm_up_open3d_cuda()

    # 이후의 메인 연산 시작
    print("Starting main computation...")
    # Initialize IMUCorrector and ICPHandler
    imu_corrector = IMUCorrector()
    experiment_folder = "./ekf_results"  # Set correct path for saving results
    icp_handler = ICPHandler(imu_corrector, experiment_folder)

    # Run both IMU and ICP
    rospy.spin()
