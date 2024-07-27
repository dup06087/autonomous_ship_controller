#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import threading
import math
import time

class ICPTest:
    def __init__(self, mother_instance):
        # rospy.init_node('icp_test')
        self.prev_scan = None
        self.mother_instance = mother_instance
        self.sub_lidar = rospy.Subscriber('/processed_pointcloud', PointCloud2, self.lidar_callback)
        # self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        
        # Initial position and heading
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_heading = 0.0  # Heading in degrees

        self.log_file = open("position_log.txt", "w")

        # Initialize previous values for tracking
        self.prev_value = { # local, inner value
            'latitude': None,
            'longitude': None,
            'heading': None,
            'pitch': None
        }

        self.flag_execute = False

        # Start the thread to update the current values
        self.update_thread = threading.Thread(target=self.update_values)
        self.update_thread.daemon = True
        self.update_thread.start()

    def run(self):
        rospy.spin()
        
    def update_values(self):
        rate = rospy.Rate(5)  # 0.2 seconds
        while not rospy.is_shutdown():
            for key in ['latitude', 'longitude', 'heading', 'pitch']:
                if not self.flag_execute:
                    value = self.mother_instance.current_value[key]
                    if value is not None:
                        self.prev_value[key] = self.mother_instance.current_value[key]
            rate.sleep()

    def lidar_callback(self, data):
        prev_time = time.time()
        # Check if the timestamp is too old
        time_diff = rospy.Time.now() - data.header.stamp
        if time_diff.to_sec() > 0.05:  # Adjust this threshold as needed
            # rospy.logwarn("Dropping old point cloud data")
            return
        try:
            # Check if any required value is None in mother_instance.current_value
            if not self.flag_execute:
                self.prev_scan = None
                self.current_heading = self.mother_instance.current_value['heading']
                self.current_x = 0
                self.current_y = 0
                self.current_z = 0
                return
            
            else:
                cloud = self.point_cloud2_to_o3d(data)
                # cloud = self.downsample(cloud)
                if self.prev_scan is not None:
                    # Perform ICP
                    reg_p2p = o3d.pipelines.registration.registration_icp(
                        cloud, self.prev_scan, 0.5,
                        np.identity(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPoint())
                    transf = reg_p2p.transformation
                    # print("fitness : ",  reg_p2p.fitness)
                    if reg_p2p.fitness > 0.8:  # Check fitness score
                        self.current_x = 0
                        self.current_y = 0
                        self.current_z = 0
                        
                        translation = transf[:3, 3]
                        rotation_matrix = transf[:3, :3]
                        rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)
                        # Update heading
                        heading_change = np.degrees(rotation_euler[2])  # Yaw change in degrees
                        print("heading change : ", heading_change)
                        self.current_heading = self.prev_value['heading'] - heading_change
                        self.current_heading = self.current_heading % 360  # Normalize heading to [0, 360)
                        # Calculate distance moved
                        self.current_x += translation[0]
                        self.current_y += translation[1]
                        self.current_z += translation[2]
                        # Print current position and heading
                        # rospy.loginfo(f"Current Position: x={self.current_x}, y={self.current_y}, z={self.current_z}")
                        # rospy.loginfo(f"Current Heading: {self.current_heading} degrees")

                        self.log_file.write(f"{rospy.Time.now().to_sec()}, {self.current_x}, {self.current_y}, {self.current_z}, {heading_change}, {self.current_heading}\n")

                        # Update mother_instance's current_value directly using ICP results
                        # if self.prev_value['latitude'] is None or self.prev_value['longitude'] is None or self.prev_value['pitch'] is None or self.prev_value['heading'] is None:
                        lat, lon = self.calculate_new_position(
                            self.prev_value['latitude'],
                            self.prev_value['longitude'],
                            self.current_x,
                            self.current_y,
                            self.current_heading
                        )
                        
                        self.prev_value['latitude'] = round(lat, 8)
                        self.prev_value['longitude'] = round(lon, 8)
                        self.prev_value['heading'] = round(self.current_heading, 2)
                        
                        self.mother_instance.current_value['latitude'] = round(lat, 8)
                        self.mother_instance.current_value['longitude'] = round(lon, 8)
                        self.mother_instance.current_value['heading'] = round(self.current_heading, 2)
                        
                        self.mother_instance.serial_gnss_cpy.current_value['latitude'] = round(lat, 8)
                        self.mother_instance.serial_gnss_cpy.current_value['longitude'] = round(lon, 8)
                        self.mother_instance.serial_gnss_cpy.current_value['heading'] = round(self.current_heading, 2)
                        
                        # print("ICP value : lat = {}, lon = {}".format(lat, lon))
                    else: 
                        print("fitness low")
                        # return # don't update if fitness is not enough and leave the prev_scan value
                    
                self.prev_scan = cloud
                # print("end lidar callback")
            print("ICP done")
        except Exception as e:
            print('lidar callback error : ', e)
            self.mother_instance.serial_gnss_cpy.flag_gnss = False
        
        print("ICP time consuming : ", time.time()-prev_time)
            
    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading):
        # Convert degrees to radians
        heading_rad = math.radians(heading + 90)

        # Calculate the change in position
        delta_north = delta_x * math.sin(heading_rad) - delta_y * math.cos(heading_rad)
        delta_east = delta_x * math.cos(heading_rad) + delta_y * math.sin(heading_rad)

        # Earth radius in meters
        R = 6378137.0

        # Calculate new latitude
        delta_lat = delta_north / R
        new_lat = lat + math.degrees(delta_lat)

        # Calculate new longitude
        delta_lon = delta_east / (R * math.cos(math.radians(lat)))
        new_lon = lon + math.degrees(delta_lon)

        return new_lat, new_lon

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
            x = np.arctan2(R[2, 1], R[2, 2]) #roll
            y = np.arctan2(-R[2, 0], sy) #pitch
            z = np.arctan2(R[1, 0], R[0, 0]) #yaw
        else: 
            x = np.arctan2(-R[1, 2], R[1, 1]) # roll
            y = np.arctan2(-R[2, 0], sy) # pitch
            z = 0 # yaw
        return np.array([x, y, z])

    def __del__(self):
        # Close the log file when the node is shut down
        self.log_file.close()

if __name__ == '__main__':
    mother_instance = None  # 여기에 mother_instance를 실제로 전달해야 함
    try:
        icp_test = ICPTest(mother_instance)
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass