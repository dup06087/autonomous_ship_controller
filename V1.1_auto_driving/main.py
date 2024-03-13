import math, queue, socket, time, threading, serial, json, random, select, re, atexit
from Jetson_initalizing_values import initialize_variables
from Jetson_serial_gnss import serial_gnss
from Jetson_serial_nucleo import serial_nucleo
from Jetson_socket_pc import JetsonSocket

# from lidar_changing import LidarProcessing

from auto_drive import auto_drive

class boat:
    def __init__(self):
        self.current_value = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", 'com_status_send': False, 'com_status_recv': False, # pc get params
            'mode_chk': "SELF", 'pwml_chk': None, 'pwmr_chk': None, # nucleo get params
            'pwml_auto': None, 'pwmr_auto': None, 'pwml_sim': None, 'pwmr_sim': None, 'cnt_destination' : None, 'distance': None, "waypoint_latitude" : None, "waypoint_longitude" : None, # auto drving
            # gnss get params below
            'velocity': None, 'heading': 0, 'roll': None, 'pitch': None, 'validity': None, 'time': None, 'IP': None, 'date': None,
            "longitude": 127.077618, "latitude": 37.633173,
            # gnss end
            } # cf. com_status undefined
        initialize_variables(self)

        self.serial_nucleo_cpy = None
        self.serial_gnss_cpy = None
        
        self.list_obstacles = None
        self.lidar_thread = None
        self.lidar_processing_cpy = None

        self.gnss_lock = threading.Lock()

        self.obstacle_coordinates = None

    def socket_LiDAR(self):
        pass
        '''
        self.current_value["waypoint_latitude"] = None
        self.current_value["waypoint_longitude"] = None
        ''' # need to define upper params here



    def gnss_thread(self):
        self.serial_gnss_cpy = None
        self.serial_gnss_cpy_thread = None

        try:
            # self.serial_gnss_cpy = serial_gnss("/dev/ttyACM0")
            self.serial_gnss_cpy = serial_gnss("/dev/septentrio2", self.gnss_lock, 1)
            self.serial_gnss_cpy_thread = threading.Thread(target=self.serial_gnss_cpy.run)
            self.serial_gnss_cpy_thread.start()
            
        except Exception as e:
            print("gnss error : ", e)
      
    def nucleo_thread(self):
        try:
            self.serial_nucleo_cpy = serial_nucleo("/dev/nucleo")
            self.serial_nucleo_cpy_thread = threading.Thread(target=self.serial_nucleo_cpy.run, args = (self.current_value,))
            self.serial_nucleo_cpy_thread.start()
        
        except Exception as e:
            print("nucleo error : ", e)

    def check_lidar_thread(self):
        while not self.obstacle_coordinates:
            time.sleep(1)
            print('waiting')
            

        try:
            self.jetson_socket_pc = JetsonSocket()
            self.recv_thread = threading.Thread(target=self.jetson_socket_pc.socket_pc_recv)
            self.send_thread = threading.Thread(target=self.jetson_socket_pc.socket_pc_send, args=(self.current_value,))
            
            self.send_obstacle_thread = threading.Thread(target=self.jetson_socket_pc.socket_pc_send_obstacle, args = (self.obstacle_coordinates))
            # send_obstacle_thread = threading.Thread(target=self.jetson_socket_pc.socket_pc_send_obstacle, args = (self.list_obstacles,))
            self.recv_thread.start()
            self.send_thread.start()
            self.send_obstacle_thread.start()
        except Exception as e:
            print("pc socket error : ", e)

    def pc_socket_thread(self):
        self.wait_thead = threading.Thread(target=self.check_lidar_thread)
        self.wait_thead.start()



    def collect_data_thread(self):
        try:
            self.collecting_data_thread = threading.Thread(target = self.collect_data)
            self.collecting_data_thread.start()
        except Exception as e:
            print("collect data init error : ", e)

    def update_seperate_data(self):
        try:
            (self.current_value["mode_chk"], self.current_value["pwml_chk"], self.current_value["pwmr_chk"])= self.serial_nucleo_cpy.processed_data
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


    def update_gnss_data(self):
        with self.gnss_lock:
            self.current_value.update(self.serial_gnss_cpy.current_value)

    def collect_data(self):
        try:    
            print("collecting")
            '''add obstacle related coordiante etc...'''
            tasks = [lambda : self.update_seperate_data(), 
                    lambda : self.current_value.update(self.jetson_socket_pc.get_value_from_pc),
                    self.update_gnss_data
            ]
            prev_time_collect_data = time.time()
            while True:
                for task in tasks:
                    try:
                        task()
                        
                    except Exception as e:
                        print("update error : {}".format(e))
                
                if time.time() - prev_time_collect_data >=1:
                    prev_time_collect_data = time.time()

                # print("collected current value : ", self.current_value)
                time.sleep(0.2)
        except:
            pass    

        

    def flag_ready_to_auto_driving(self):
        try:
            flag_ready_devices = all(self.jetson_socket_pc.flag_pc_recv_alive, self.jetson_socket_pc.flag_pc_send_alive, self.serial_nucleo_cpy.flag_nucleo_alive, self.serial_gnss_cpy.flag_gnss_alive) # need to add lidar
            flag_ready_data = (self.current_value['latitude'] is not None and self.current_value['longitude'] is not None and
                        self.current_value['dest_latitude'] is not None and self.current_value['dest_longitude'] is not None and self.current_value['heading'] is not None)
            flag_auto_drive_command =(self.current_value["mode_pc_command"] == "AUTO")
            flag_ready_to_auto_drive = flag_ready_data and flag_ready_devices and flag_auto_drive_command
        except Exception as e:
            # print("flag ready to auto driving error : ", e)
            return (False, None, None, None, None, None)
        
        return (flag_ready_to_auto_drive, self.current_value['latitude'], self.current_value['longitude'], self.current_value['dest_latitude'], self.current_value['dest_longitude'], self.current_value['heading'])

    def auto_driving(self):
        try:
            self.auto_drive = auto_drive
            self.auto_drive_thread = threading.Thread(target= self.auto_drive, args = (self,))
            self.auto_drive_thread.start()
        
        except Exception as e:
            print("auto_driving error : ", e)

    def get_obstacle_coordinates(self):
        client_socket = None
        data_buffer = b''
        while True: 
            try:
                print("running")
                if not client_socket:
                    host = 'localhost'
                    port = 5000
                    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    client_socket.connect((host,port))

                    if client_socket is not None:
                        try: 
                            _, ready_to_read, _ = select.select([client_socket], [client_socket], [], 1)

                            try:
                                if ready_to_read:
                                    data = client_socket.recv(1024)
                                    if data:
                                        data_buffer += data
                                        if b'\n' in data_buffer:
                                            received_list = json.loads(data_buffer.decode('utf-8'))
                                            print("received list : ", received_list)
                            except Exception as e:
                                print(e)
                        except Exception as e:
                            print(e)
            except Exception as e:
                print(e)

    def get_obstacle_coordinates_thread(self):
        self.get_obstacle = threading.Thread(target = self.get_obstacle_coordinates)
        self.get_obstacle.start()

    def thread_start(self):
        # self.lidar_thread()
        self.gnss_thread()
        self.nucleo_thread()
        self.get_obstacle_coordinates_thread()
        self.collect_data_thread()
        self.pc_socket_thread()
        self.auto_driving()

        # time.sleep(5)
        
        # self.lidar_thread = self.lidar_processing_cpy
        # while True:
        #     # self.collect_data_main()
            
        #     '''auto driving part'''
        #     # if self.current_value["mode_pc_command"] == "AUTO" and self.flag_ready_to_auto_drving == False:
        #     #     self.current_value["mode_pc_command"] = "SELF"

        #     if time.time() - self.prev_time_print >= 5:
        #         print("program running in main thread")
        #         self.prev_time_print = time.time()
            
        #     time.sleep(2)

Boat = boat()
Boat.thread_start()