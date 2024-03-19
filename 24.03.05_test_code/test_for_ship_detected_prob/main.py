import math, queue, socket, time, threading, serial, json, random, select, re, atexit
from Jetson_initalizing_values import initialize_variables
from Jetson_serial_gnss import serial_gnss
from Jetson_serial_nucleo import serial_nucleo
from Jetson_socket_pc import Server_pc
from Jetson_lidar_execution import LidarProcessor

from auto_drive import auto_drive

import copy

class boat:
    def __init__(self):
        self.current_value = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", 'com_status_send': False, 'com_status_recv': False, 
            "coeff_kf" : 3.5, "coeff_kd" : 0.6, "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : None,
            # pc get params
            'mode_chk': "SELF", 'pwml_chk': None, 'pwmr_chk': None, # nucleo get params
            'pwml_auto': None, 'pwmr_auto': None, 'pwml_sim': None, 'pwmr_sim': None, 'cnt_destination' : 0, 'distance': None, "waypoint_latitude" : None, "waypoint_longitude" : None, # auto drving
            # gnss get params below
            'velocity': None, 'heading': 0, 'roll': None, 'pitch': None, 'validity': None, 'time': None, 'IP': None, 'date': None,
            "longitude": 127.077618, "latitude": 37.633173,
            "arrived" : False, "flag_autodrive" : False
            # gnss end
            } # cf. com_status undefined
        initialize_variables(self)

        self.serial_nucleo_cpy = None
        self.serial_gnss_cpy = None
        self.lidar_processing_cpy = None

        self.gnss_lock = threading.Lock()

        # self.obstacle_coordinates = None

    def gnss_thread(self):
        self.serial_gnss_cpy = None
        self.serial_gnss_cpy_thread = None

        try:
            # self.serial_gnss_cpy = serial_gnss("/dev/ttyACM0")
            # self.serial_gnss_cpy = serial_gnss("/dev/tty_septentrio0", self.gnss_lock, 1)
            self.serial_gnss_cpy = serial_gnss("/dev/pts/4", self.gnss_lock, 1)
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
        
    def collect_data(self):
        try:    
            # print("collecting")
            '''add obstacle related coordiante etc...'''
            tasks = [lambda : self.update_seperate_data(), 
                    self.update_pc_command,
                    self.update_pc_coeff,
                    self.update_jetson_coeff,
                    self.update_vff_coeff,
                    self.update_gnss_data

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
            pass        


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

        self.send_pitch_to_lidar(pitch = self.current_value['pitch'])
    
    def send_pitch_to_lidar(self, pitch):
        self.lidar_processor.pitch = pitch
        # self.lidar_processor.pitch = random.randint(0,90)
        
    def flag_check_for_autodrive(self):
        flag_devices = self.flag_check(self.flag_devices()) #True or False
        
        flag_data = self.data_check_all_device()
        
        
        self.flag_autodrive = flag_devices and flag_data[0] ### final ready flag
        if self.flag_autodrive == False and self.current_value["mode_pc_command"] == "AUTO":
            print("some device not ready : ", self.flag_devices())
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
        flags = self.jetson_socket_pc.flag_socket_pc, self.lidar_processor.flag_lidar, self.serial_nucleo_cpy.flag_nucleo_alive, self.serial_gnss_cpy.flag_gnss
        # print("flags : ", flags)
        
        return flags
    
    def data_check_all_device(self):
        lat = self.current_value['latitude']
        lon = self.current_value['longitude']
        dest_lat = self.current_value['dest_latitude']
        dest_lon = self.current_value['dest_longitude']
        heading = self.current_value['heading']
        
        flag_ready_gnss_data = (lat is not None and lon is not None and dest_lat is not None and dest_lon is not None and heading is not None)
        flag_ready_nucleo_data = (not self.current_value["mode_chk"] <= 20) # mode_chk == 0 >> need to bind
        flag_auto_drive_command =(self.current_value["mode_pc_command"] == "AUTO")
        print(lat, lon, dest_lat, dest_lon, heading)
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
        self.lidar_processor = LidarProcessor()
        self.lidar_processor_thread = threading.Thread(target = self.lidar_processor.run)
        self.lidar_processor_thread.start()
        # lidar_processor.bbox_lists
        
    def thread_start(self):
        self.lidar_thread()
        self.gnss_thread()
        self.nucleo_thread()
       
        self.collect_data_thread()
        self.pc_socket_thread()
        self.auto_driving() # block in while
        
        time_prev = 0
        while True:
            time_ = time.time()
            if time_ - time_prev >= 3:
                try:
                    print("from Nucleo : ", self.serial_nucleo_cpy.nucleo_feedback_values)
                    print("to Nucleo : ", self.serial_nucleo_cpy.nucleo_sended_data)
                except:
                    pass
                try:
                    print("lidar processing time : ", self.lidar_processor.lidar_processing_time, ", first data : ", self.lidar_processor.bbox_lists[0])
                except:
                    pass
                
                try:
                    print("from pc : ", self.jetson_socket_pc.pc_command)
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
                    print("flag for autodriving : [(nucleo,gps,command), lat, lon, dest_lat, dest_lon, heading]", self.flag_check_for_autodrive())
                except:
                    pass
                    
                
                time_prev = time_
            
            time.sleep(0.05) # if time.sleep doesn't exit > time overloading
            
                
Boat = boat()
Boat.thread_start()