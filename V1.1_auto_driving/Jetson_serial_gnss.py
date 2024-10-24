import serial, threading, time, atexit
import threading
import serial
from queue import Queue
import time
# import Jetson_gps_send_to_gnss

class serial_gnss:
    def __init__(self, port, lock, id, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=2)

        self.receive_queue = Queue()
        self.running = True

        self.current_value = {'validity' : None, 'latitude' : None, 'longitude' : None, 'velocity' : None, 'date' :  None, 'time' : None, 'heading' : None, 'pitch' : None}

        # self.sender_to_lidar = Jetson_gps_send_to_gnss.UdpBroadcaster(10110)

        self.flag_gnss_alive = False

        self.gnss_lock = lock
        self.id = id
        

    def run(self):
        self.receive_thread = threading.Thread(target=self._data_receive_part)
        self.receive_thread.start()
        # self.receive_thread.join()
        print("receiving thread started")

        self.process_receive_thread = threading.Thread(target=self._data_processing_part)
        self.process_receive_thread.start()
        # self.process_receive_thread.join()
        print("data processing thread started")
        
    
    def close(self):
        self.running = False
        self.serial_port.close()

    def add_to_queue(self, data):
        with self.gnss_lock:
            self.receive_queue.put(data)

    def get_from_queue(self):
        if not self.receive_queue.empty():
            return self.receive_queue.get()
        return None

    def _data_receive_part(self):
        while self.running:
            # print("running")
            try:
                # print(self.id)
                time.sleep(0.2)  # '''time control'''
                lines = []

                while self.serial_port.in_waiting:
                    try:
                        line = self.serial_port.readline()
                        if line:
                            lines.append(line)
                    except Exception as e:
                        print("readline error : ", e) 
                        # print("gnss variable line : ", line)  ## must be one value

                try:
                    if len(lines) >=3:
                        line_ = [lines[-2], lines[-1]]
                    elif len(lines) == 2:
                        line_ = lines
                    elif len(lines) == 1:
                        line_ = lines
                    else:
                        print("empty lines : length : ", len(lines))
                        line_ = ''

                    if line_:
                        self.add_to_queue(line_)
                except Exception as e:
                    print("gnss in lines check : ", lines, line_)

            except Exception as e:
                print("GNSS Error : ", e)
                # lines = []
                # self.close()
                
    def _data_processing_part(self):
        while self.running:
            try:
                time.sleep(0.2)     #'''time control'''
                data = self.get_from_queue()
                # print("1. gnss data : ", data)
                if data:
                    self.process_to_single_data(data)

            except Exception as e:
                print(f"Error processing data: {e}")

    def process_to_single_data(self, data_list):
        for data in data_list:
            self.process_received_data(data)

    def process_received_data(self, data):
        
        try:
            decoded_data = data.decode('utf-8').strip()
            # print("2. decoded_data : ", decoded_data)
            if not decoded_data.startswith('$'):
                time.sleep(0.2)
                return

            tokens = decoded_data.split(',')
            # print("tokens : ", tokens)
            header = tokens[0]

            if header in ['$GPRMC', '$GNRMC']:
                self._process_gnrmc_data(tokens)
                # self.send_to_lidar(decoded_data)
                # print("gnrmc_data processing done")
            elif header == '$PSSN':  # HRP
                self._process_pssn_data(tokens)
                # print("pssn_data processing done")
            else:
                print("gnss header : ", header)
                print("GNSS 데이터 오류 발생 : ", header)
                time.sleep(0.2)
                return

        except Exception as e:
            print(f"Error processing gnss data: {e}")

    def send_to_lidar(self, GPS_data):
        self.sender_to_lidar.send_data(GPS_data)
        # print("sended to lidar")

    def _process_gnrmc_data(self, tokens):
        try:
            validity = tokens[2]
            if validity == "V": # V : invalid, A : valid
                self.current_value['validity'] = validity
                self.flag_gnss_alive = False
                # print("validity : ", validity)
                # print("current validity : ", self.current_value["validity"])
                # print("inner current value : ", self.current_value)
                return

            self.current_value['validity'] = validity
            lat_min = float(tokens[3])
            lat_deg = int(lat_min / 100)
            lat_min -= lat_deg * 100
            self.current_value['latitude'] = round(lat_deg + lat_min / 60, 8)

            lon_sec = float(tokens[5])
            lon_deg = int(lon_sec / 100)
            lon_min = (lon_sec / 100 - lon_deg) * 100
            self.current_value['longitude'] = round(lon_deg + lon_min / 60, 8)

            self.current_value['velocity'] = float(tokens[7])
            self.flag_gnss_alive = True # _process_gnrmc_data valid or not

        except ValueError as e:
            print(f"Error processing GNRMC data: {e}")


    def _process_pssn_data(self, tokens):
        try:
            self.current_value['time'] = tokens[2]
            self.current_value['date'] = tokens[3]
            self.current_value['heading'] = float(tokens[4])
            self.current_value['pitch'] = float(tokens[6])

            self.flag_auto_ready_gnss = True

        except ValueError as e:
            # when heading pitch is not comming heading pitch raw data comes '' not None
            # print(f"Error heading pitch processing PSSN data: {e}")
            self.current_value['heading'] = None
            self.current_value['pitch'] = None
    
            self.flag_auto_ready_gnss = False

if __name__ == "__main__":
    serial_cpy = None
    serial_cpy_thread = None
    cnt = 0
    gnss_lock =  threading.Lock()
    try:
        # serial_cpy = serial_gnss("/dev/ttyACM0")
        serial_cpy = serial_gnss("/dev/septentrio2", gnss_lock, 1)
        serial_cpy_thread = threading.Thread(target=serial_cpy.run)
        serial_cpy_thread.start()
        
    except Exception as e:
        print("gnss error : ", e)
            
    while True:
        print("running")
        time.sleep(1)
        # try:
            
        #     try:
        #         try:
        #             print("gnss current value : ",serial_cpy.current_value)
        #         except Exception as e:
        #             print("gnss log : ", e)
        #         if serial_cpy.process_receive_thread.is_alive() == False:
        #             del serial_cpy
        #             del serial_cpy_thread  
        #             serial_cpy = None
        #             serial_cpy_thread = None

        #     except:
        #         # serial_cpy = serial_gnss("/dev/ttyACM0")
        #         serial_cpy = serial_gnss("/dev/septentrio2", gnss_lock, 2)
        #         serial_cpy_thread = threading.Thread(target=serial_cpy.run)
        #         serial_cpy_thread.start()
                
        # except Exception as e:
        #     print("error 2 : ", e)
                
        # time.sleep(2)