import socket
import threading
import json
import time

class Server_pc:
    def __init__(self, boat_instance, receive_coeff_port= 5002, receive_port=5003, send_port=5004, send_obstacle_port=5005):
        self.boat = boat_instance
        self.receive_port = receive_port
        self.receive_coeff_port = receive_coeff_port
        self.send_port = send_port
        self.send_obstacle_port = send_obstacle_port
        self.send_data = {"example_key": "example_value"}
        self.send_obstacle_data = {"obstacle": "example_obstacle"}
        self.pc_command = {'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF"}
        self.pc_coeff = {"coeff_kv_p" : 3.5, "coeff_kv_i" : 0.6, "coeff_kv_d" : 3.5, "coeff_kw_p" : 0.6, "coeff_kw_i" : 3.5, "coeff_kw_d" : 0.6, "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : 0}
        self.flag_socket_pc = [False, False, False, False] # receive, send_data, send_obstacle, coeff
        self.flag_socket_init_cycle = [False, False, False, False]

    def receive_messages(self, receive_socket):
        last_print_time = time.time()
        while self.flag_run_thread:
            try:
                data = receive_socket.recv(1024).strip()
                if not data: # client unconnected
                    print("receive_message no data")
                    self.flag_socket_pc[0] = False
                    break
                
                if data == b'\x00':
                    continue
                
                try:
                    received_dict = json.loads(data.decode('utf-8'))
                    self.pc_command.update(received_dict)

                    
                    self.flag_socket_pc[0] = True
                    receive_socket.sendall("ack".encode())  # Acknowledgment 메시지 보내기
                    
                    

                except (json.JSONDecodeError, TypeError, ValueError) as e:
                    current_time = time.time()
                    if current_time - last_print_time >= 3:
                        print("Invalid data format: ", data)
                        last_print_time = current_time

                self.flag_socket_init_cycle[0] = True
                time.sleep(0.05)    
            except (ConnectionResetError, BrokenPipeError) as e:
                print("Receive connection lost.", e)
                self.flag_socket_pc[0] = False
                break
            
            except Exception as e:
                print("received message outter error : ", e)
        print("receive thread end")
    
    def receive_coeff_messages(self, receive_socket):
        last_print_time = time.time()
        while self.flag_run_thread:
            # print("coeff running?")
            try:
                data = receive_socket.recv(1024).strip()
                # print("data : ", data)
                if not data:
                    print("coeff receive no data : thread end")
                    self.flag_socket_pc[3] = False
                    break
                    
                if data == b'\x00':
                    # print("check")
                    continue
                
                try:
                    received_dict = json.loads(data.decode('utf-8'))
                    self.pc_coeff.update(received_dict)
                    self.flag_socket_pc[3] = True
                    receive_socket.sendall("ack".encode())  # Acknowledgment 메시지 보내기
                    # print("coeff : ", self.pc_coeff)

                except (json.JSONDecodeError, TypeError, ValueError) as e:
                    print("coeff except")
                    current_time = time.time()
                    if current_time - last_print_time >= 3:
                        print("Invalid data format: ", data, e)
                        last_print_time = current_time

                self.flag_socket_init_cycle[3] = True
                time.sleep(0.05)

            except (ConnectionResetError, BrokenPipeError) as e:
                print("Receive connection lost.", e)
                self.flag_socket_pc[3] = False
                break
            
            except Exception as e:
                print("receive coeff error ", e)
                
    def send_messages(self, send_data, send_socket):
        self.message_to_pc = {}
        while self.flag_run_thread:
            time.sleep(0.2)
            try:
                self.message_to_pc = json.dumps(self.boat.current_value)
                self.message_to_pc += '\n'
                send_socket.sendall(self.message_to_pc.encode())
                self.flag_socket_pc[1] = True
                self.flag_socket_init_cycle[1] = True

            except (BrokenPipeError, ConnectionResetError) as e:
                print("Send connection lost:", e)
                self.flag_socket_pc[1] = False
                break

    def send_obstacle_messages(self, send_socket):
        while self.flag_run_thread:
            time.sleep(0.2)
            try:
                message = json.dumps(self.boat.lidar_processor.bbox_lists)
                send_socket.send((message + '\n').encode())
                self.flag_socket_pc[2] = True
            except BrokenPipeError as e:
                print("Send obstacle data connection lost. Attempting to reconnect...", e)
                self.flag_socket_pc[2] = False
                break
            except ConnectionResetError as e:
                self.flag_socket_pc[2] = False
                print("Connection for obstacle data was reset by the server. Attempting to reconnect...", e)
                break
            except Exception as e:
                print("data error : ", e)
            
            self.flag_socket_init_cycle[2] = True


    def run(self):
        while True:
            try:
                print("regenerating")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_coeff_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_obstacle_socket:
                    receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    receive_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                    receive_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)

                    receive_coeff_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    receive_coeff_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    receive_coeff_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                    receive_coeff_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)
                    send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    send_obstacle_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                    receive_socket.bind(('0.0.0.0', self.receive_port))
                    receive_socket.listen(1)
                    print(f"Receiving server listening on port {self.receive_port}")
                    
                    receive_coeff_socket.bind(('0.0.0.0', self.receive_coeff_port))
                    receive_coeff_socket.listen(1)
                    print(f"Receiving coeffserver listening on port {self.receive_coeff_port}")

                    send_socket.bind(('0.0.0.0', self.send_port))
                    send_socket.listen(1)
                    print(f"Sending server listening on port {self.send_port}")
                    
                    send_obstacle_socket.bind(('0.0.0.0', self.send_obstacle_port))
                    send_obstacle_socket.listen(1)
                    print(f"Sending obstacle server listening on port {self.send_obstacle_port}")

                    # 각 소켓에 대한 타임아웃 설정
                    receive_socket.settimeout(5)
                    receive_coeff_socket.settimeout(5)
                    send_socket.settimeout(5)
                    send_obstacle_socket.settimeout(5)

                    # 각 소켓에 대해 무한 대기 설정
                    while True:
                        try:
                            receive_client, _ = receive_socket.accept()
                            # receive_client.settimeout(2)
                            print("Accepted receive connection")
                            break
                        except socket.timeout:
                            print("Waiting for receive connection...")
                            continue
                        
                    while True:
                        try:
                            receive_coeff_client, _ = receive_coeff_socket.accept()
                            print("Accepted receive coeff connection")
                            break
                        except socket.timeout:
                            print("Waiting for recive coeff connection...")
                            continue

                    while True:
                        try:
                            send_client, _ = send_socket.accept()
                            print("Accepted send connection")
                            break
                        except socket.timeout:
                            print("Waiting for send connection...")
                            continue

                    while True:
                        try:
                            send_obstacle_client, _ = send_obstacle_socket.accept()
                            print("Accepted send obstacle connection")
                            break
                        except socket.timeout:
                            print("Waiting for send obstacle connection...")
                            continue

                    receive_thread = threading.Thread(target=self.receive_messages, args=(receive_client,))
                    receive_coeff_thread = threading.Thread(target=self.receive_coeff_messages, args=(receive_coeff_client,))
                    send_thread = threading.Thread(target=self.send_messages, args=(self.send_data, send_client))
                    send_obstacle_thread = threading.Thread(target=self.send_obstacle_messages, args=(send_obstacle_client,))

                    self.flag_run_thread = True                    
                    receive_thread.start()
                    receive_coeff_thread.start()
                    send_thread.start()
                    send_obstacle_thread.start()
                    
                    while True:
                        if all(self.flag_socket_init_cycle):
                            print("break")
                            break
                                    
                    while all(self.flag_socket_pc):
                        print("pc communicating well")
                        time.sleep(10)
                        
                    print("thread end regenerating")
                    self.flag_run_thread = False                    
                    receive_socket.close()
                    receive_coeff_socket.close()
                    send_socket.close()
                    send_obstacle_socket.close()
                    # self.flag_run_thread = False 
                    self.flag_socket_pc = [False, False, False, False] 
                    self.flag_socket_init_cycle = [False, False, False, False] 
                    
                    time.sleep(2)
                    
            except Exception as e:
                print("socket thread error : ",e)
                time.sleep(1)

if __name__ == "__main__":
    from main import boat
    boat_ = boat()
    jetson_socket = Server_pc(boat_)
    jetson_socket_thread = threading.Thread(target=jetson_socket.run)
    jetson_socket_thread.start()
