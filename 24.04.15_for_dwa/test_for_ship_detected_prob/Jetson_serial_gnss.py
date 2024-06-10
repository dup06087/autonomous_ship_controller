import serial, threading, time, atexit
import threading
from queue import Queue
import time
from datetime import datetime
import atexit

class serial_gnss:
    def __init__(self, port, lock, id, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=10)

        self.receive_queue = Queue()
        self.running = True

        self.current_value = {'validity' : None, 'latitude' : None, 'longitude' : None, 'velocity' : None, 'date' :  None, 'time' : None, 'heading' : None, 'pitch' : None}

        # self.sender_to_lidar = Jetson_gps_send_to_gnss.UdpBroadcaster(10110)

        self.flag_gnss = False
        self.cnt_receive = 0
        self.cnt_process = 0
        
        self.gnss_lock = lock
        self.id = id
        
        self.lpf_flag = True
        self.alpha = 0.2/(1+0.2)  # LPF 가중치 설정
        self.previous_values = {'latitude': None, 'longitude': None, 'heading': None, 'pitch': None}

        atexit.register(self.close_serial)

    def run(self):
        self.receive_thread = threading.Thread(target=self._data_receive_part)
        self.receive_thread.start()
        # self.receive_thread.join()
        print("gnss receiving thread started")

        self.process_receive_thread = threading.Thread(target=self._data_processing_part)
        self.process_receive_thread.start()
        # self.process_receive_thread.join()
        print("gnss data processing thread started")
        
    def close_serial(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            with open("close_serial.txt", "a") as file:
                file.write("closed gnss well\n")
            print("gnss port closed.")
            
    def apply_low_pass_filter(self, value_type, new_value):
        if self.lpf_flag:  # lpf_flag가 True인 경우에만 LPF 적용
            if new_value is None:
                return None # 새 값이 None인 경우 필터링 하지 않고 반환
            if self.previous_values[value_type] is None:
                filtered_value = new_value
            else:
                filtered_value = self.alpha * new_value + (1 - self.alpha) * self.previous_values[value_type]
            self.previous_values[value_type] = filtered_value
            return filtered_value
        else:
            return new_value  # lpf_flag가 False인 경우 새 값을 그대로 반환
        
    def close(self):
        self.running = False
        self.serial_port.close()

    def add_to_queue(self, data):
        with self.gnss_lock:
            # print("gnss data : ", data)
            self.receive_queue.put(data)

    def get_from_queue(self):
        if not self.receive_queue.empty():
            return self.receive_queue.get()
        return None

    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=10)
        except Exception as e:
            print(f"Failed to open serial port {self.port}: {e}")
            
    def _data_receive_part(self):
        waiting = False
        while self.running:
            try:
                time.sleep(0.2)  # '''time control'''
                lines = []
                
                if not self.serial_port.is_open:
                    time.sleep(1)  # Wait before retrying
                    self.connect_serial()
                    print("restart gnss serial connection")
                    continue
                
                while self.serial_port.in_waiting:
                    try:
                        # print("waiting")
                        line = self.serial_port.readline()
                        # print("waiting end")
                        if line:
                            # print("line : ", line)
                            lines.append(line)
                    except serial.SerialException as e:
                        print("gnss serial exception : ", e, " >> : gnss flag False")
                        for key in self.current_value.keys():
                            self.current_value[key] = None
                        self.flag_gnss = False
                        time.sleep(1)
                        
                    except Exception as e:
                        
                        print("gnss readline error : ", e) 
                        print("gnss variable line : ", line)  ## must be one value

                try:
                    if len(lines) >=3:
                        line_ = [lines[-2], lines[-1]]
                        # print("here12")
                    elif len(lines) == 2:
                        line_ = lines
                        # print("here13")
                    elif len(lines) == 1:
                        line_ = lines
                        # print("here14")
                    else:
                        # print("empty lines : length : ", len(lines))
                        line_ = ''

                    if line_:
                        # print("here12123")
                        self.add_to_queue(line_)
                                                    
                except Exception as e:
                
                    print("gnss in lines check : ", lines, line_)

            except Exception as e:
                print("GNSS Error data receive part : ", e)
                # lines = []
                # self.close()
                
    def _data_processing_part(self):
        count_alive = 0
        while self.running:
            try:
                # print("gnss error cnt : ", self.cnt_receive, self.cnt_process, self.flag_gnss)
                time.sleep(0.2)     #'''time control'''
                data = self.get_from_queue()
                # print("1. gnss data : ", data)
                if data:
                    self.process_to_single_data(data)
                    count_alive = 0
                    # print("gnss serial count ", count_alive)
                else:
                    count_alive += 1
                    if count_alive >= 6:
                        print("gnss false here1, cnt : ", count_alive)
                        self.flag_gnss = False
                    else:
                        self.flag_gnss = True

            except Exception as e:
                print(f"Error processing data: {e}")

    def process_to_single_data(self, data_list):
        for data in data_list:
            self.process_received_data(data)

    def process_received_data(self, data):
        try:
            decoded_data = data.decode('utf-8').strip()
            
            
            current_time = datetime.now()
            log_time = current_time.strftime("%m-%d %H:%M:%S") + '.' + str(current_time.microsecond // 100000)
            
            '''throttling problem too big data'''
            with open("log_gnss_raw.txt", 'a') as file:
                file.write(f"{log_time} : {decoded_data}\n")
            
            # print("2. decoded_data : ", decoded_data)
            if not decoded_data.startswith('$'):
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
                self.cnt_receive += 1
                self.current_value['validity'] = validity
                if self.cnt_receive >= 5:
                    print("gnss false here 2")
                    self.flag_gnss = False
                # print("validity : ", validity)
                # print("current validity : ", self.current_value["validity"])
                # print("inner current value : ", self.current_value)
                return

            self.current_value['validity'] = validity
            lat_min = float(tokens[3])
            lat_deg = int(lat_min / 100)
            lat_min -= lat_deg * 100

            lon_sec = float(tokens[5])
            lon_deg = int(lon_sec / 100)
            lon_min = (lon_sec / 100 - lon_deg) * 100
            
            new_lat = round(lat_deg + lat_min / 60, 8)
            new_lon = round(lon_deg + lon_min / 60, 8)

            # LPF 적용
            # filtered_lat = self.apply_low_pass_filter('latitude', new_lat)
            # filtered_lon = self.apply_low_pass_filter('longitude', new_lon)

            self.current_value['latitude'] = round(new_lat, 8)
            self.current_value['longitude'] = round(new_lon, 8)

            self.current_value['velocity'] = float(tokens[7])
            self.cnt_receive = 0

        except ValueError as e:
            print(f"Error processing GNRMC data: {e}")


    def _process_pssn_data(self, tokens):
        try:
            self.current_value['time'] = tokens[2]
            self.current_value['date'] = tokens[3]
            new_heading = float(tokens[4])
            new_pitch = float(tokens[6])

            # LPF 적용
            # filtered_heading = self.apply_low_pass_filter('heading', new_heading)
            # filtered_pitch = self.apply_low_pass_filter('pitch', new_pitch)

            self.current_value['heading'] = round(new_heading, 2)
            self.current_value['pitch'] = round(new_pitch, 2)

            self.flag_gnss = True
            self.cnt_process = 0

        except ValueError as e:
            # when heading pitch is not comming heading pitch raw data comes '' not None
            # print(f"heading pitch not came: {e}")
    
            self.cnt_process += 1
            if self.cnt_process >= 5:
                self.current_value['heading'] = None
                self.current_value['pitch'] = None
                print("gnss false : heading, pitch")
                self.flag_gnss = False
            
        except Exception as e:
            print("processing pssn error : ", e)
        
        

if __name__ == "__main__":
    serial_cpy = None
    serial_cpy_thread = None
    cnt = 0
    gnss_lock =  threading.Lock()
    try:
        # serial_cpy = serial_gnss("/dev/ttyACM2") # origin name
        serial_cpy = serial_gnss("/dev/tty_septentrio0", gnss_lock, 1) # tty fixed name
        # serial_cpy = serial_gnss("/dev/pts/5", gnss_lock, 1) # Virtual Port 
        serial_cpy_thread = threading.Thread(target=serial_cpy.run)
        serial_cpy_thread.start()
        
    except Exception as e:
        print("gnss error : ", e)
            
    while True:
        print(serial_cpy.current_value)
        time.sleep(1)
        