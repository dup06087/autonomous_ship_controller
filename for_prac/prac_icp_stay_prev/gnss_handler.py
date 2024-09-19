import serial
import os
import threading
import time
from datetime import datetime
import rospy

class GNSSHandler:
    def __init__(self, experiment_folder):
        self.experiment_folder = experiment_folder
        self.gnss_data_file = os.path.join(self.experiment_folder, "gnss_raw.txt")
        self.current_value = {
            'validity': None, 
            'latitude': None, 
            'longitude': None, 
            'velocity': None, 
            'date': None, 
            'time': None, 
            'heading': None, 
            'pitch': None
        }
        self.serial_port = serial.Serial("/dev/ttyACM1", 115200, timeout=10)
        # self.serial_port = serial.Serial("/dev/pts/7",115200,timeout=10)

        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        while self.running:
            try:
                time.sleep(1)
                lines = []
                while self.serial_port.in_waiting:
                    line = self.serial_port.readline()
                    if line:
                        lines.append(line)

                if lines:
                    self.process_to_single_data(lines)
            except Exception as e:
                print(f"Error receiving GNSS data: {e}")

    def close(self):
        self.running = False
        self.thread.join()
        if self.serial_port.is_open:
            self.serial_port.close()
            print("GNSS port closed.")

    def process_to_single_data(self, data_list):
        with open(self.gnss_data_file, 'a') as f:
            for data in data_list:
                self.process_received_data(data, f)

    def process_received_data(self, data, file):
        try:
            decoded_data = data.decode('utf-8').strip()
            log_time = rospy.Time.now().to_sec()
            file.write(f"{log_time} : {decoded_data}\n")
            
            if not decoded_data.startswith('$'):
                return

            tokens = decoded_data.split(',')
            header = tokens[0]

            if header in ['$GPRMC', '$GNRMC', '$GARMC']:
                self._process_gnrmc_data(tokens)
            elif header == '$PSSN':
                self._process_pssn_data(tokens)
            else:
                pass
                # print(f"Unrecognized header: {header}")

        except Exception as e:
            pass
            # print(f"Error processing GNSS data: {e}")

    def _process_gnrmc_data(self, tokens):
        try:
            validity = tokens[2]
            if validity == "V":
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

    def save_data(self):
        # 필요시 데이터를 저장하는 추가 로직을 여기에 추가할 수 있습니다.
        pass
