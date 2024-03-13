import serial, threading, time, atexit
from queue import Queue
import rospy
import std_msgs.msg

class serial_nucleo:
    def __init__(self, port, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.4)

        # self.motor_pwm_log = rospy.Publisher('/motor_output_log', std_msgs.msg.String, queue_size=None, tcp_nodelay=True)
        # string_msg = std_msgs.msg.String()
        # string_msg.data = data
        # self.motor_pwm_log.publish(string_msg)
        
        self.receive_queue = Queue()
        self.transmit_queue = Queue()
        self.running = True

        self.receive_thread = threading.Thread(target=self.data_receive_part)
        self.receive_thread.start()
        print("receiving thread started")

        self.process_receive_thread = threading.Thread(target=self.data_processing_part)
        self.process_receive_thread.start()
        print("data processing thread started")

        self.process_transmit_thread = threading.Thread(target=self.data_transmission_part)
        self.process_transmit_thread.start()

        self.lock_recv = threading.Lock()
        self.lock_send = threading.Lock()

        self.flag_nucleo_alive = {"recv" : False, "send" : False}
        self.mode_pc_command = None
        self.pwmr_auto = None
        self.pwml_auto = None
        self.nucleo_feedback_values = None
        self.nucleo_sended_data = None

        # neutral : 'mode_chk': 0, 'pwml_chk': 1500, 'pwmr_chk': 1200

    def add_to_queue(self, data):
        with self.lock_recv:
            self.receive_queue.put(data)
        
    def get_from_queue(self):
        with self.lock_recv:
            if not self.receive_queue.empty():
                return self.receive_queue.get()
            return None

    # below get from nucleo
    def data_receive_part(self):
        while self.running:
            try:
                time.sleep(0.2)
                lines = []
                while self.serial_port.in_waiting:
                    line = self.serial_port.readline()
                    # print("raw data : ", line)
                    if line:
                        lines.append(line)
                data = lines[-1] if lines else None

                if data:
                    self.add_to_queue(data)


            except Exception as e:
                print(f"Error while receiving data: {e}")
                # self.flag_nucleo_alive = False

    def data_processing_part(self):
        while self.running:
            try:
                time.sleep(0.05)
                data = self.get_from_queue()
                if data:
                    self.nucleo_feedback_values = self.process_received_data(data)
                    self.flag_nucleo_alive["recv"] = True
                    # print(f"received Data from nucleo: {self.nucleo_feedback_values}")
            except Exception as e:
                print(f"Error processing data: {e}")

    # queue에서 받아온 것 처리해서 쓸모있는 것으로
    def process_received_data(self, data):
        nucleo_values = "mode:-1,PWML:-1,PWMR:-1"
        decoded_data = data.decode('utf-8').strip()
        if "mode:" in decoded_data and "PWML:" in decoded_data and "PWMR:" in decoded_data:
            parsed_data = dict(item.split(":") for item in decoded_data.split(","))
            mode_chk = int(parsed_data.get('mode', '-1').strip()) # PERR : parse error
            pwml_chk = int(parsed_data.get('PWML', '0').strip())
            pwmr_chk = int(parsed_data.get('PWMR', '0').strip())
            # print("parsed : {}, {}, {} ".format(parsed_mode, parsed_pwml, parsed_pwmr))
            nucleo_values = mode_chk, pwml_chk, pwmr_chk
            # print("get data from nucleo : ", nucleo_values)
            
        else:
            print("Received unexpected data : ", data)

        return nucleo_values
    
    # below transmission to nucleo
    def data_transmission_part(self):
        while self.running:
            try:
                time.sleep(0.2)
                with self.lock_send:
                    data = self.transmit_queue.get()
                    # print("data : ", data)
                    if data:
                        self.nucleo_sended_data = self.prepare_data_for_transmission(data)
                        self.serial_port.write(self.nucleo_sended_data)
                        self.flag_nucleo_alive["send"] = True


            except Exception as e:
                print(f"Error while transmitting data: {e}")


    def prepare_data_for_transmission(self, sending_data):
        # print("nucleo data check ",sending_data)
        keys = ["mode_pc_command", "pwmr_auto", "pwml_auto"]
        try:
            if all(key in sending_data for key in keys):
                self.mode_pc_command = sending_data['mode_pc_command']
                self.pwmr_auto = sending_data["pwmr_auto"]
                self.pwml_auto = sending_data["pwml_auto"]

            data_str = f"mode:{self.mode_pc_command},PWML:{self.pwml_auto},PWMR:{self.pwmr_auto}\n"
            
            # print("sent to nucleo : ", data_str)
            return data_str.encode()
        except Exception as e:
            print("!!! : ", e)

    
    def run(self, sending_data):
        # print('runned')
        while self.running:
            time.sleep(0.2)
            try:
                # print("sending : ", sending_data)

                self.send_data(sending_data)
            except Exception as e:
                print(f"Nucleo communication error: {e}")
                time.sleep(2)


    def send_data(self, data):
        self.transmit_queue.put(data)

if __name__ == "__main__":
    # serial_nucleo("COM7")
    serial_nucleo_cpy = None
    current_value = {
        # dest_latitude, dest_longitude : list, connected with pc def start_driving
        'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", 'com_status_send': False, 'com_status_recv': False, # pc get params
        'mode_chk': "SELF", 'pwml_chk': None, 'pwmr_chk': None, # nucleo get params
        'pwml_auto': None, 'pwmr_auto': None, 'pwml_sim': None, 'pwmr_sim': None, 'cnt_destination' : None, 'distance': None, "waypoint_latitude" : None, "waypoint_longitude" : None, # auto drving
        # gnss get params below
        'velocity': None, 'heading': 0, 'roll': None, 'pitch': None, 'validity': None, 'time': None, 'IP': None, 'date': None,
        "longitude": 127.077618, "latitude": 37.633173,
        # gnss end
        } # cf. com_status undefined
        
    try:
        # serial_nucleo_cpy = serial_nucleo("/dev/ttyACM2")
        serial_nucleo_cpy = serial_nucleo("/dev/nucleo")
        serial_nucleo_cpy_thread = threading.Thread(target=serial_nucleo_cpy.run, args = (current_value,))
        serial_nucleo_cpy_thread.start()
        
    except Exception as e:
        print("nucleo error : ", e)
        

            
    
    # while True:
        
    #     try:
    #         try:
    #             # print("out call : ",serial_nucleo_cpy.processed_data)

    #             if serial_nucleo_cpy.receive_thread.is_alive() == False:
    #                 del serial_nucleo_cpy
    #                 del serial_nucleo_cpy_thread  
    #                 serial_nucleo_cpy = None
    #                 serial__nucleo_cpy_thread = None
    #                 print("thread regenerated")
    #                 print("out call : ",serial_nucleo_cpy.processed_data)


    #         except:
    #             print("out call : ",serial_nucleo_cpy.processed_data)

    #             serial_nucleo_cpy = serial_nucleo("/dev/nucleo")
    #             serial_nucleo_cpy_thread = threading.Thread(target=serial_nucleo_cpy.run, args = (current_value,))
                                                            
    #             serial_nucleo_cpy_thread.start()

    #         # print("thread alive? : ", serial_nucleo_cpy.receive_thread.is_alive())
            
    #         # print("3 : ", serial_nucleo_cpy.mode_pc_command, serial_nucleo_cpy.pwml_auto, serial_nucleo_cpy.pwmr_auto)
    #     except Exception as e:
    #         print("error 2 : ", e)
                
    #     time.sleep(0.2)