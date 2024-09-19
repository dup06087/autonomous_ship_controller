import numpy as np
import rospy
from sensor_msgs.msg import Imu, PointCloud2
from filterpy.kalman import KalmanFilter
import open3d as o3d
import math
import sensor_msgs.point_cloud2 as pc2
import open3d.core as o3c
import time

class ImuLidarKalmanFilterNode:
    def __init__(self):
        rospy.init_node('imu_lidar_kalman_filter_node')

        # 칼만 필터 초기화
        self.kf = KalmanFilter(dim_x=6, dim_z=3)  # 상태 벡터 크기 6 (p_x, p_y, theta, v_x, v_y, omega_z), 관측 벡터 크기 3 (x_icp, y_icp, heading)

        # 상태 전이 행렬 (State Transition Matrix)
        dt = 0.1  # 시간 간격 (예: 10ms)
        self.kf.F = np.array([[1, 0, 0, dt, 0,  0],
                              [0, 1, 0, 0,  dt, 0],
                              [0, 0, 1, 0,  0,  dt],
                              [0, 0, 0, 1,  0,  0],
                              [0, 0, 0, 0,  1,  0],
                              [0, 0, 0, 0,  0,  1]])

        # 관측 모델 행렬 (Measurement Matrix)
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],  # x_icp 측정값
                              [0, 1, 0, 0, 0, 0],  # y_icp 측정값
                              [0, 0, 1, 0, 0, 0]])  # heading_change 측정값

        # 초기 상태 추정 (Initial state estimate)
        self.kf.x = np.zeros(6)

        # 공분산 행렬 (Covariance Matrix)
        self.kf.P *= 1000.  # 초기 불확실성
        self.kf.R = np.eye(3) * 0.01  # 측정 불확실성 (LiDAR의 노이즈)
        self.kf.Q = np.eye(6) * 0.1  # 프로세스 노이즈

        # 로그 파일 초기화
        self.log_file = open("position_log.txt", "w")

        # IMU 및 LiDAR 데이터 구독자 설정
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)

        # 초기화
        self.last_time = rospy.Time.now()
        self.prev_scan = None
        self.prev_heading_change = 0
        self.prev_x_change = 0

    def lidar_callback(self, data):
        try:
            prev_time = time.time()
            ros_time_now = rospy.get_rostime()
            time_diff = ros_time_now - data.header.stamp    
            if time_diff.to_sec() > 0.1:
                print("f : ", time_diff.to_sec())
                return

            cloud = self.point_cloud2_to_o3d(data)

            min_bound = np.array([-10, -10, -0.5])
            max_bound = np.array([10, 10, 2])
            cloud = self.crop_roi(cloud, min_bound, max_bound)
            ship_body_bounds = {'min': [-2, -2, -0.6], 'max': [2, 2, 0.31]}
            cloud = self.remove_ship_body(cloud, ship_body_bounds)
            cloud = self.downsample(cloud)

            if self.prev_scan is not None:
                theta = np.radians(self.kf.x[2])  # 현재 헤딩 값
                self.icp_initial_guess = np.array([[np.cos(theta), -np.sin(theta), 0, self.kf.x[0]],
                                                [np.sin(theta), np.cos(theta),  0, self.kf.x[1]],
                                                [0,             0,              1, 0],
                                                [0,             0,              0, 1]])

                reg_gicp = o3d.t.pipelines.registration.icp(
                    source=cloud, 
                    target=self.prev_scan,
                    max_correspondence_distance=1.5,
                    init_source_to_target=self.icp_initial_guess,
                    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(
                        relative_fitness=1e-6,
                        relative_rmse=1e-6,
                        max_iteration=10
                    )
                )
                transf = reg_gicp.transformation.cpu().numpy()

                if reg_gicp.fitness > 0.2:
                    translation = transf[:3, 3]
                    rotation_matrix = transf[:3, :3]
                    rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)

                    # LiDAR로부터 측정된 위치와 헤딩
                    x_icp = translation[0]
                    y_icp = translation[1]
                    heading = np.degrees(rotation_euler[2])

                    # ICP 좌표계를 IMU 좌표계로 변환
                    x_east = x_icp * np.cos(np.radians(heading)) - -y_icp * np.sin(np.radians(heading))
                    y_east = x_icp * np.sin(np.radians(heading)) + -y_icp * np.cos(np.radians(heading))

                    # LiDAR 측정값을 사용해 칼만 필터 업데이트
                    z = np.array([x_east, y_east, heading])
                    self.kf.update(z)

                    # 필터링된 결과를 출력
                    p_x, p_y = self.kf.x[0], self.kf.x[1]
                    rospy.loginfo(f"Filtered Position: x={p_x:.2f}, y={p_y:.2f}")
                    self.log_file.write(f"{p_x},{p_y}\n")

            self.prev_scan = cloud

        except Exception as e:
            rospy.logerr(f"ICP error: {e}")

    def imu_callback(self, msg):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # RPY 구하기 (Quaternion to Euler)
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # 선형 가속도 보정 (Linear Acceleration Correction)
        linear_acceleration = msg.linear_acceleration
        accel_corrected = self.rotate_acceleration(
            linear_acceleration.x, 
            linear_acceleration.y, 
            linear_acceleration.z, 
            roll, pitch, yaw
        )

        # 상태 벡터 업데이트 (가속도를 사용해 속도 업데이트)
        # 현재 속도 갱신 (v_x, v_y)
        self.kf.x[3] += accel_corrected[0] * delta_time  # v_x 업데이트
        self.kf.x[4] += accel_corrected[1] * delta_time  # v_y 업데이트

        # 현재 헤딩 (yaw) 고려하여 속도를 x, y 축으로 변환하여 위치를 업데이트
        heading = self.kf.x[2]  # 현재 헤딩 (theta)
        cos_heading = np.cos(heading)
        sin_heading = np.sin(heading)

        # 속도를 현재 헤딩 방향에 맞게 회전 변환
        delta_x = (self.kf.x[3] * cos_heading - -self.kf.x[4] * sin_heading) * delta_time
        delta_y = (self.kf.x[3] * sin_heading + -self.kf.x[4] * cos_heading) * delta_time

        # 위치 업데이트 (p_x, p_y)
        self.kf.x[0] += delta_x  # p_x 업데이트
        self.kf.x[1] += delta_y  # p_y 업데이트

        # 각속도를 사용해 헤딩 업데이트
        self.kf.x[2] += msg.angular_velocity.z * delta_time  # 헤딩(theta) 업데이트

        # 칼만 필터 예측 단계
        self.kf.predict()
        
    def quaternion_to_euler(self, x, y, z, w):
        # 쿼터니언을 오일러 각도로 변환하는 함수
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

    def downsample(self, cloud, voxel_size=0.1):
        return cloud.voxel_down_sample(voxel_size)

    def remove_ship_body(self, pcd, ship_body_bounds):
        # GPU에서 CPU로 데이터를 이동
        pcd_tensor = pcd.point.positions.to(o3c.Device("CPU:0"))

        # ship_body_bounds의 min과 max 값을 텐서로 변환
        min_bound = np.array(ship_body_bounds['min'])
        max_bound = np.array(ship_body_bounds['max'])

        # 범위에 포함되지 않는 포인트를 남김
        mask = ~((pcd_tensor[:, 0].numpy() >= min_bound[0]) &
                (pcd_tensor[:, 0].numpy() <= max_bound[0]) &
                (pcd_tensor[:, 1].numpy() >= min_bound[1]) &
                (pcd_tensor[:, 1].numpy() <= max_bound[1]) &
                (pcd_tensor[:, 2].numpy() >= min_bound[2]) &
                (pcd_tensor[:, 2].numpy() <= max_bound[2]))

        filtered_pcd_tensor = pcd_tensor.numpy()[mask]
        # 필터링된 포인트들을 GPU로 다시 전송
        filtered_pcd = o3d.t.geometry.PointCloud(o3c.Tensor(filtered_pcd_tensor, dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return filtered_pcd

    def crop_roi(self, cloud, min_bound, max_bound):
        min_bound_tensor = o3c.Tensor(min_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        max_bound_tensor = o3c.Tensor(max_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        bbox = o3d.t.geometry.AxisAlignedBoundingBox(min_bound_tensor, max_bound_tensor)
        cropped_cloud = cloud.crop(bbox)
        return cropped_cloud

    def point_cloud2_to_o3d(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(np.array(points), dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return cloud

    def rotation_matrix_to_euler(self, R):
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def __del__(self):
        self.log_file.close()

if __name__ == '__main__':
    try:
        node = ImuLidarKalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
