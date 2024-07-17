import math, queue, socket, time, threading, serial, json, random, select, re, atexit, rospy
from Jetson_initalizing_values import initialize_variables
from Jetson_serial_gnss import serial_gnss
from Jetson_serial_nucleo import serial_nucleo
from Jetson_socket_pc import Server_pc
from Jetson_lidar_execution import PointCloudProcessor
# from prac_ICP import ICPTest

# from auto_drive import auto_drive
from auto_drive_PID import auto_drive
# from auto_drive_pwm_diff_correction import auto_drive
from goal_publish import NavigationController
from datetime import datetime

import copy
from math import radians, cos, sin, asin, sqrt, atan2, degrees
from nav_msgs.msg import Path

import signal
import sys
from costmap_gps_publisher import GPSPublisher

import os

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class boat:
    def __init__(self):
        self.current_value = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", 'com_status_send': False, 'com_status_recv': False, 
            "coeff_kf" : 3.5, "coeff_kd" : 0.6, "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : 0,
            # pc get params
            'mode_chk': "SELF", 'pwml_chk': None, 'pwmr_chk': None, # nucleo get params
            'pwml_auto': None, 'pwmr_auto': None, 'pwml_sim': None, 'pwmr_sim': None, 'cnt_destination' : 0, 'distance': None, "waypoint_latitude" : None, "waypoint_longitude" : None, # auto drving
            "waypoint_lat_m" : None, "waypoint_lon_m" : None,
            # gnss get params below
            'velocity': None, 'heading': 0, 'roll': None, 'pitch': None, 'validity': None, 'time': None, 'IP': None, 'date': None,
            "longitude": 127.077618, "latitude": 37.633173, "obstacle_cost" : 0,
            "arrived" : False, "flag_autodrive" : False
            # gnss end
            } # cf. com_status undefined
        
        initialize_variables(self)

        self.serial_nucleo_cpy = None
        self.serial_gnss_cpy = None
        self.lidar_processing_cpy = None

        self.gnss_lock = threading.Lock()
        # self.path_publisher = rospy.Publisher("/filtered_global_plan", Path, queue_size=2)

        # self.obstacle_coordinates = None

        self.log_folder_path = self.create_log_folder()


    def create_log_folder(self):
        # Create folder path based on the current year, month-date, and time
        current_time = datetime.now()
        year = current_time.strftime("%Y")
        month_date = current_time.strftime("%m-%d")
        time = current_time.strftime("%H%M")
        folder_path = os.path.join("log", year, month_date, time)
        
        # Ensure the folder exists
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print("log file maked : {}".format(folder_path))
        
        return folder_path
    
    def haversine(self, lon1, lat1, lon2, lat2):
        """
        Calculate the great circle distance in meters between two points 
        on the earth (specified in decimal degrees)
        """
        # Convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

        # Haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        r = 6371000 # Radius of earth in meters. Use 3956 for miles
        return c * r

    def calculate_bearing(self, lon1, lat1, lon2, lat2):
        """
        Calculate the bearing between two points on the earth
        """
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        initial_bearing = atan2(x, y)
        # Now we have the initial bearing but math.atan2 return values from -π to +π (−180° to +180°)
        # So we normalize the result by converting it to a compass bearing from 0° to 360°
        initial_bearing = degrees(initial_bearing)
        bearing = (initial_bearing + 360) % 360
        return bearing

    def coordinate_transform(self, distance, bearing, heading):
        """
        Transform the target point relative to the ship's current heading
        """
        # Convert heading and bearing to radians for calculation
        heading_rad = radians(heading)
        bearing_rad = radians(bearing)
        
        # Calculate relative angle
        relative_angle = bearing_rad - heading_rad
        
        # Transform to x, y coordinates
        x = distance * cos(relative_angle)
        y = distance * sin(relative_angle)
        
        return x, y

    def gnss_thread(self):
        self.serial_gnss_cpy = None
        self.serial_gnss_cpy_thread = None

        try:
            self.serial_gnss_cpy = serial_gnss("/dev/ttyACM1", self.gnss_lock, 1, self)
            # sudo chmod a+rw /dev/ttyACM0
            # self.serial_gnss_cpy = serial_gnss("/dev/tty_septentrio0", self.gnss_lock, 1, self)
            # self.serial_gnss_cpy = serial_gnss("/dev/pts/10", self.gnss_lock, 1, self)
            self.serial_gnss_cpy_thread = threading.Thread(target=self.serial_gnss_cpy.run)
            self.serial_gnss_cpy_thread.start()
            print("gnss started well")
        except Exception as e:
            print("gnss error : ", e)
      
    def nucleo_thread(self):
        try:
            self.serial_nucleo_cpy = serial_nucleo("/dev/nucleo")
            self.serial_nucleo_cpy_thread = threading.Thread(target=self.serial_nucleo_cpy.run, args = (self.current_value,))
            self.serial_nucleo_cpy_thread.start()
        
        except Exception as e:
            print("nucleo error : ", e)

    # def ICP_thread(self):
    #     try:
    #         self.icp_test_cpy = ICPTest(self)
    #         self.icp_test_cpy_thread = threading.Thread(target=self.icp_test_cpy.run)
    #         self.icp_test_cpy_thread.start()
    #     except Exception as e:
    #         print(f"Exception in nucleo_thread: {e}")
        
    #     except Exception as e:
    #         print("nucleo error : ", e)
            
        



    def pc_socket_thread(self):
        try:
            self.jetson_socket_pc = Server_pc(self)
            self.jetson_socket_pc_thread = threading.Thread(target=self.jetson_socket_pc.run)
            self.jetson_socket_pc_thread.start()
            
        except Exception as e:
            print("pc socket error : ", e)

    def collect_data_thread(self):
        try:
            self.collecting_data_thread = threading.Thread(target = self.collect_data)
            self.collecting_data_thread.start()
        except Exception as e:
            print("collect data init error : ", e)
    
    def update_local_waypoint(self, data):
        try:
            if data.poses:
                last_pose = data.poses[-1]
                self.current_value["waypoint_lat_m"] = round(float(last_pose.pose.position.x), 2)
                self.current_value["waypoint_lon_m"] = round(float(last_pose.pose.position.y), 2)
                
                t = time.localtime()    
                log_time = time.strftime("%H:%M:%S", t)
                file_path = os.path.join(self.log_folder_path, "log_global_planner.txt")
        
                with file_path as file:
                    file.write("{} : {}, {}\n".format(log_time, self.current_value["waypoint_lat_m"], self.current_value["waypoint_lon_m"]))
                
                print(f"Updated waypoint: d_lat = {self.current_value['waypoint_lat_m']}, d_lon = {self.current_value['waypoint_lon_m']}")
        except Exception as e:
            print("Error in update_waypoint: ", e)

    def update_global_waypoint(self, data):
        try:
            if not self.flag_stop_update_waypoint:
                if data.poses:
                    self.last_received_time = data.header.stamp.to_sec()

                    # print("globaldata : \n", data)
                    try:
                        last_pose = data.poses[10]
                    except:
                        last_pose = data.poses[-1]
                        
                    self.current_value["waypoint_lat_m"] = round(float(last_pose.pose.position.x), 2)
                    self.current_value["waypoint_lon_m"] = round(float(last_pose.pose.position.y), 2)
                    
                    t = time.localtime()    
                    log_time = time.strftime("%H:%M:%S", t)
                    with open("log_global_planner", "a") as file:
                        file.write("{} : {}, {}\n".format(log_time, self.current_value["waypoint_lat_m"], self.current_value["waypoint_lon_m"]))
                    
                    print(f"Updated waypoint: d_lat = {self.current_value['waypoint_lat_m']}, d_lon = {self.current_value['waypoint_lon_m']}")
                    # self.path_publisher.publish(data)

                else:
                    pass
            else:
                print("ignore current global waypoint(cleared but not updated costmap)")
                
        except Exception as e:
            self.flag_waypoint_publishing = False
            print("Error in update_waypoint: ", e)

    def check_global_waypoint_timeout(self, event):
        if self.last_received_time is None:
            return  # 아직 메시지를 받지 않았다면 패스

        if self.current_value['flag_autodrive'] == False:
            return
        else:
            current_time = rospy.get_time()
            if current_time - self.last_received_time > self.goal_subscribe_timeout: ### last_received_time은 update_global_waypoint에서 초기화
                self.current_value['pwml_auto'] = 1500
                self.current_value['pwmr_auto'] = 1500
                rospy.loginfo("긴급 중지: 경로 업데이트 안 됨")
            
    def collect_data(self):
        try:    
            # print("collecting")
            '''add obstacle related coordiante etc...'''
            tasks = [lambda : self.update_seperate_data(), 
                    self.update_pc_command,
                    self.update_pc_coeff,
                    self.update_jetson_coeff,
                    # self.update_vff_coeff,
                    self.update_gnss_data,
            ]
            prev_time_collect_data = time.time()
            while True:
                for task in tasks:
                    try:
                        task()
                        
                    except Exception as e:
                        if time.time() - prev_time_collect_data >=1:
                            prev_time_collect_data = time.time()
                            print("collect data error : {}".format(e))
                        # pass
      
                # print("collected current value : ", self.current_value)
                
                ''' current value logger '''
                # print('log_current value doing??')
                current_time = datetime.now()
                log_time = current_time.strftime("%m-%d %H:%M:%S") + '.' + str(current_time.microsecond // 100000)
                file_path = os.path.join(self.log_folder_path, "log_current_value.txt")

                with open(file_path, 'a') as file:
                    file.write(f"{log_time} : {self.current_value}\n")
                    # print("done?")
                time.sleep(0.2)
                
        except Exception as e:
            print("data collecting error : ", e)

    def update_vff_coeff(self):
        try:
            self.coeff_vff_repulsive_force = self.jetson_socket_pc.pc_coeff["coeff_vff_repulsive_force"]
        except Exception as e:
            print("vff repulsive coeff receive error : ", e)
            
    def update_seperate_data(self):
        try:
            (self.current_value["mode_chk"], self.current_value["pwml_chk"], self.current_value["pwmr_chk"])= self.serial_nucleo_cpy.nucleo_feedback_values
        except Exception as e:
            pass
            
        try:
            self.current_value["com_status_recv"] = self.jetson_socket_pc.flag_pc_recv_alive
        except Exception as e:
            pass
        try:
            self.current_value["com_status_send"] = self.jetson_socket_pc.flag_pc_send_alive
        except Exception as e:
            pass
        
        try:
            self.current_value["flag_autodrive"] = self.flag_autodrive
        except Exception as e:
            print("flag_autodrive update error : ", e)        

    def update_pc_command(self):
        current_pc_command = self.jetson_socket_pc.pc_command
        if current_pc_command != self.prev_pc_command:
            self.current_value.update(current_pc_command)
            self.prev_pc_command = copy.deepcopy(current_pc_command)

    def update_pc_coeff(self):
        current_pc_coeff = self.jetson_socket_pc.pc_coeff
        if current_pc_coeff != self.prev_pc_coeff:
            self.current_value.update(current_pc_coeff)
            self.prev_pc_coeff = copy.deepcopy(current_pc_coeff)
    
    def update_jetson_coeff(self):
        self.lidar_processor.update_coeff(self.current_value["coeff_kd"], self.current_value["coeff_kd"], self.current_value["voxel_size"],
                                          self.current_value["intensity"],self.current_value["dbscan_eps"],self.current_value["dbscan_minpoints"],self.current_value["coeff_vff_repulsive_force"])
            
    def update_gnss_data(self):
        with self.gnss_lock:
            self.current_value.update(self.serial_gnss_cpy.current_value)

        # GPS 데이터를 업데이트
        gps_data = {
            'latitude': self.current_value['latitude'],
            'longitude': self.current_value['longitude'],
            'heading': self.current_value['heading']
        }
        self.gps_publisher.update_gps_data(gps_data)  # GPS 데이터를 GPSPublisher에 전달

        self.send_pitch_to_lidar(pitch = self.current_value['pitch'])
    
    
    
    
    def send_pitch_to_lidar(self, pitch):
        self.lidar_processor.pitch = pitch
        # self.lidar_processor.pitch = random.randint(0,90)
        
    def flag_check_for_autodrive(self):
        self.autodrive_output_flag = self.flag_devices()
        flag_devices = self.flag_check(self.autodrive_output_flag) #True or False
        
        flag_data = self.data_check_all_device()
        
        
        self.flag_autodrive = flag_devices and flag_data[0] ### final ready flag
        if self.flag_autodrive == False and self.current_value["mode_pc_command"] == "AUTO":
            print("some device not ready : ", self.autodrive_output_flag)
        # print("flag1 : ", flag_devices, ", flag_data : ", flag_data, ", flag_autodrive : ", flag_autodrive)
        flag_output = [self.flag_autodrive, flag_data[1], flag_data[2], flag_data[3], flag_data[4], flag_data[5]]
        
        return flag_output
        # [ready or not(bool), lat, lon, dest_lat, dest_lon, heading]

    def flag_check(self, flags):
        for flag in flags:
            # If the flag is a list, check if all elements in the list are True
            if isinstance(flag, list):
                if not all(flag):
                    return False
            # If the flag is a boolean, directly check its value
            elif isinstance(flag, bool):
                if not flag:
                    return False
            # If the flag is a dictionary, check if all values are True
            elif isinstance(flag, dict):
                if not all(flag.values()):
                    return False
            else:
                # Unsupported data type
                return False
        return True

    def flag_devices(self):
        flags = self.jetson_socket_pc.flag_socket_pc, self.serial_nucleo_cpy.flag_nucleo_alive, self.serial_gnss_cpy.flag_gnss
        # print("flags : ", flags)    
        return flags
    
    def data_check_all_device(self):
        lat = self.current_value['latitude']
        lon = self.current_value['longitude']
        dest_lat = self.current_value['dest_latitude']
        dest_lon = self.current_value['dest_longitude']
        heading = self.current_value['heading']
        
        flag_ready_gnss_data = (lat is not None and lon is not None and dest_lat is not None and dest_lon is not None and heading is not None)
        flag_ready_nucleo_data = (not self.current_value["mode_chk"] <= 20) # mode_chk == 0 >> need to bind # error : '<=' not supported between instances of 'str' and 'int'
        flag_auto_drive_command =(self.current_value["mode_pc_command"] == "AUTO")
        # print(lat, lon, dest_lat, dest_lon, heading)
        # print("flag_ready_gnss_data : ", flag_ready_gnss_data, ", flag_ready_nucleo_data : ", flag_ready_nucleo_data, ", flag_auto_drive_command : ", flag_auto_drive_command)
        return [flag_ready_nucleo_data and flag_ready_gnss_data and flag_auto_drive_command, lat, lon, dest_lat, dest_lon, heading]
        # [ready or not(bool), lat, lon, dest_lat, dest_lon, heading]
        
    def auto_driving(self):
        try:
            self.auto_drive = auto_drive
            self.auto_drive_thread = threading.Thread(target= self.auto_drive, args = (self,))
            self.auto_drive_thread.start()
        
        except Exception as e:
            print("auto_driving error : ", e)

    def lidar_thread(self):
        self.lidar_processor = PointCloudProcessor()
        self.lidar_processor_thread = threading.Thread(target = self.lidar_processor.run)
        self.lidar_processor_thread.start()
        # lidar_processor.bbox_lists
    
    def goal_publishing_thread(self):
        self.goal_controller = NavigationController(self)
        self.goal_controller_thread = threading.Thread(target = self.goal_controller.publish_nav_goal)
        self.goal_controller_thread.start()
        print("goal_controller started")
        
    def thread_start(self):
        prev_pc_command = None
        self.lidar_thread()
        self.gnss_thread()
        self.nucleo_thread()
        # self.ICP_thread()
        self.collect_data_thread()
        self.pc_socket_thread()

        
        self.goal_publishing_thread()
        self.gps_publisher = GPSPublisher()  # GPSPublisher 인스턴스 생성
        self.gps_publisher.start()  # GPS 데이터를 publish하는 스레드 시작
        
        
        self.auto_driving() # block in while
        
        time_prev = 0
        while True:
            time_ = time.time()
            if time_ - time_prev >= 3:
                try:
                    pass
                    # print("from Nucleo : ", self.serial_nucleo_cpy.nucleo_feedback_values)
                    # print("to Nucleo : ", self.serial_nucleo_cpy.nucleo_sended_data)
                except:
                    pass
                try:
                    print("lidar processing time : ", self.lidar_processor.lidar_processing_time, ", first data : ", self.lidar_processor.bbox_lists[0])
                except:
                    pass
                
                try:
                    if prev_pc_command != self.jetson_socket_pc.pc_command:
                        print("from pc : ", self.jetson_socket_pc.pc_command)
                        prev_pc_command = self.jetson_socket_pc.pc_command
                except:
                    pass
                
                try:
                    print("to pc current value: ", self.jetson_socket_pc.message_to_pc)
                except:
                    pass
                
                try:
                    print("output : ", self.current_value)
                except:
                    pass
                
                try:
                    pass
                    # print("flag for autodriving : [(nucleo,gps,command), lat, lon, dest_lat, dest_lon, heading]", self.flag_check_for_autodrive())
                except:
                    pass
                    
                
                time_prev = time_
            
            time.sleep(0.05) # if time.sleep doesn't exit > time overloading
            
                
Boat = boat()
Boat.thread_start()