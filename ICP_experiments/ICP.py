#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import open3d.core as o3c
import threading
import math
import time

import serial
import time
from datetime import datetime
import os

class serial_gnss:
    def __init__(self, port="/dev/ttyACM1", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=10)
        
        self.current_value = {
            'validity': None, 
            'latitude': None, 
            'longitude': None, 
            'velocity': None, 
            'date':  None, 
            'time': None, 
            'heading': None, 
            'pitch': None
        }

    def run_once(self):
        try:
            # 데이터를 한번만 읽어오기
            lines = []
            while self.serial_port.in_waiting:
                line = self.serial_port.readline()
                if line:
                    lines.append(line)

            if len(lines) > 0:
                self.process_to_single_data(lines)
            
        except Exception as e:
            print(f"Error receiving data: {e}")
        finally:
            self.close_serial()
    
    def close_serial(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            print("gnss port closed.")
            
    def process_to_single_data(self, data_list):
        for data in data_list:
            self.process_received_data(data)

    def process_received_data(self, data):
        try:
            decoded_data = data.decode('utf-8').strip()
            current_time = datetime.now()
            log_time = current_time.strftime("%m-%d %H:%M:%S") + '.' + str(current_time.microsecond // 100000)
            
            print(f"{log_time} : {decoded_data}")
            
            if not decoded_data.startswith('$'):
                return

            tokens = decoded_data.split(',')
            header = tokens[0]

            if header in ['$GPRMC', '$GNRMC']:
                self._process_gnrmc_data(tokens)
            elif header == '$PSSN':  # HRP
                self._process_pssn_data(tokens)
            else:
                print(f"Unrecognized header: {header}")

        except Exception as e:
            print(f"Error processing gnss data: {e}")

    def _process_gnrmc_data(self, tokens):
        try:
            validity = tokens[2]
            if validity == "V":  # V : invalid, A : valid
                self.current_value['validity'] = validity
                return

            self.current_value['validity'] = validity
            lat_min = float(tokens[3])
            lat_deg = int(lat_min / 100)
            lat_min -= lat_deg * 100

            lon_sec = float(tokens[5])
            lon_deg = int(lon_sec / 100)
            lon_min = (lon_sec / 100 - lon_deg) * 100
            
            self.current_value['latitude'] = round(lat_deg + lat_min / 60, 8)
            self.current_value['longitude'] = round(lon_deg + lon_min / 60, 8)
            self.current_value['velocity'] = float(tokens[7])

        except ValueError as e:
            print(f"Error processing GNRMC data: {e}")

    def _process_pssn_data(self, tokens):
        try:
            self.current_value['time'] = tokens[2]
            self.current_value['date'] = tokens[3]
            self.current_value['heading'] = round(float(tokens[4]), 2)
            self.current_value['pitch'] = round(float(tokens[6]), 2)

        except ValueError as e:
            print(f"Error processing PSSN data: {e}")
            
            
class ICPTest:
    def __init__(self, mother_instance):
        self.prev_scan = None
        self.mother_instance = mother_instance
        self.sub_lidar = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)

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

        self.prev_value = { # local, inner value
            'latitude': serial_cpy.current_value['latitude'],
            'longitude': serial_cpy.current_value['longitude'],
            'heading': serial_cpy.current_value['heading'],
            'pitch': serial_cpy.current_value['pitch']
        }
        
        self.flag_execute = True
        
    def run(self):
        rospy.spin()

    def lidar_callback(self, data):
        prev_time = time.time()
        # Check if the timestamp is too old
        time_diff = rospy.Time.now() - data.header.stamp
        if time_diff.to_sec() > 0.05:  # Adjust this threshold as needed
            print("erase old pc data")
            return
        try:
            # Check if any required value is None in mother_instance.current_value
            if self.flag_execute:
                cloud = self.point_cloud2_to_o3d(data)

                # Define ROI (e.g., a box from (-20, -20, -1) to (20, 20, 1))
                min_bound = np.array([-20, -20, -1])
                max_bound = np.array([20, 20, 1])
                
                # # Crop ROI
                cloud = self.crop_roi(cloud, min_bound, max_bound)

                # Downsample point cloud
                cloud = self.downsample(cloud)

                if self.prev_scan is not None:
                    # Perform ICP
                    reg_p2p = o3d.pipelines.registration.registration_icp(
                        cloud.to_legacy(), self.prev_scan.to_legacy(), 0.5,
                        np.identity(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPoint())
                    transf = reg_p2p.transformation

                    if reg_p2p.fitness > 0.5:  # Check fitness score
                        self.current_x = 0
                        self.current_y = 0
                        self.current_z = 0
                        
                        translation = transf[:3, 3]
                        rotation_matrix = transf[:3, :3]
                        rotation_euler = self.rotation_matrix_to_euler(rotation_matrix)
                        
                        heading_change = np.degrees(rotation_euler[2])  # Yaw change in degrees
                        heading_change = math.trunc(heading_change * 10) / 10
                        print("heading_change : ", heading_change)
                        self.current_heading = self.prev_value['heading'] - heading_change
                        if self.current_heading > 180:
                            self.current_heading -= 360
                            
                        self.current_heading = self.current_heading % 360  # Normalize heading to [0, 360)

                        # Calculate distance moved
                        floored_lat = self.floor_to_eight_decimal_places(translation[0])
                        floored_lon = self.floor_to_eight_decimal_places(translation[1])

                        self.current_x += floored_lat
                        self.current_y += floored_lon

                        # Log current position and heading

                        # Update mother_instance's current_value directly using ICP results
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
                        
                        print("current value : ", self.prev_value['latitude'], self.prev_value['longitude'], self.prev_value['heading'])
                        self.log_file.write(f"{rospy.Time.now().to_sec()}, {self.current_x}, {self.current_y}, {self.current_z}, {heading_change}, {self.current_heading}\n")

                        
                    else: 
                        print("fitness low")
                        
                self.prev_scan = cloud
                
        except Exception as e:
            print('lidar callback error : ', e)
            # self.mother_instance.serial_gnss_cpy.flag_gnss = False
        
        # print("ICP time consuming : ", time.time()-prev_time)

    def floor_to_eight_decimal_places(self, value):
        return math.trunc(value * 10**2) / 10**2

    def crop_roi(self, cloud, min_bound, max_bound):
        # GPU에서 ROI 자르기 수행
        min_bound_tensor = o3c.Tensor(min_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        max_bound_tensor = o3c.Tensor(max_bound, dtype=o3c.float32, device=cloud.point.positions.device)
        bbox = o3d.t.geometry.AxisAlignedBoundingBox(min_bound_tensor, max_bound_tensor)
        cropped_cloud = cloud.crop(bbox)
        return cropped_cloud
            
    def calculate_new_position(self, lat, lon, delta_x, delta_y, heading):
        # Convert degrees to radians
        if heading > 180:
            heading -= 360
            
        heading_rad = math.radians(-heading)
        delta_north = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        delta_east = -delta_x * math.sin(heading_rad) - delta_y * math.cos(heading_rad)

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
        cloud = o3d.t.geometry.PointCloud(o3c.Tensor(np.array(points), dtype=o3c.float32, device=o3c.Device("CUDA:0")))
        return cloud

    def downsample(self, cloud, voxel_size=0.1):
        # GPU에서 다운샘플링 수행
        return cloud.voxel_down_sample(voxel_size)

    def rotation_matrix_to_euler(self, rotation_matrix):
        R = np.array(rotation_matrix)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2]) # roll
            y = np.arctan2(-R[2, 0], sy) # pitch
            z = np.arctan2(R[1, 0], R[0, 0]) # yaw
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
        serial_cpy = serial_gnss()
        serial_cpy.run_once()
        print(serial_cpy.current_value)
        
        rospy.init_node("pointcloud_processor", anonymous=True)
        icp_test = ICPTest(mother_instance)
        icp_test.run()
    except rospy.ROSInterruptException:
        pass
