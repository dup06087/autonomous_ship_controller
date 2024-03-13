import socket
import threading
import time
import json

class Server_pc:
    def __init__(self, boat_instance, receive_port=5003, send_port=5004, send_obstacle_port=5005):
        self.boat = boat_instance
        
        self.receive_port = receive_port
        self.send_port = send_port
        self.send_obstacle_port = send_obstacle_port
        self.send_data = {"example_key": "example_value"}  # Example data to send
        self.send_obstacle_data = {"obstacle": "example_obstacle"}  # Example obstacle data
        self.pc_command = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", # pc get params
        }
        
        self.flag_socket_pc = [False, False, False]

    def receive_messages(self, receive_socket):
        last_print_time = time.time()
        while True:
            try:
                data = receive_socket.recv(1024).strip()
                if not data:
                    break
                try:
                    received_dict = json.loads(data.decode('utf-8'))
                    self.pc_command.update(received_dict)
                    # print("PC >> Jetson", self.pc_command)
                    
                    self.flag_socket_pc[0] = True
                    
                except (json.JSONDecodeError, TypeError, ValueError):
                    current_time = time.time()
                    if current_time - last_print_time >= 1:
                        print("Waiting for pc command")
                        last_print_time = current_time
            except (ConnectionResetError, BrokenPipeError) as e:
                print("Receive connection lost.", e)
                self.flag_socket_pc[0] = False
                break

    def send_messages(self, send_data, send_socket):
        self.message_to_pc = {}
        while True:
            time.sleep(0.2)
            try:
                # message = json.dumps(send_data)
                self.message_to_pc = json.dumps(self.boat.current_value)
                self.message_to_pc += '\n'
                send_socket.sendall(self.message_to_pc.encode())
                self.flag_socket_pc[1] = True
                # print("Sent:", message)
            except (BrokenPipeError, ConnectionResetError) as e:
                print("Send connection lost:", e)
                self.flag_socket_pc[1] = False
                break

    def send_obstacle_messages(self, send_socket):
        self.message_to_pc_obstacle = []
        while True:
            time.sleep(0.2)
            try:
                self.message_to_pc_obstacle = json.dumps(self.boat.lidar_processor.bbox_lists)
                send_socket.send(self.message_to_pc_obstacle.encode())
                self.flag_socket_pc[2] = True
                # print(f"Sent obstacle data: {message}")
            except BrokenPipeError as e:
                print("Send obstacle data connection lost. Attempting to reconnect...", e)
                self.flag_socket_pc[2] = False
                break
            except ConnectionResetError:
                self.flag_socket_pc[2] = False
                print("Connection for obstacle data was reset by the server. Attempting to reconnect...")
                break
            
    def run(self):
        while True:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_socket, \
                    socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_socket, \
                    socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_obstacle_socket:
                
                receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                send_obstacle_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                receive_socket.bind(('0.0.0.0', self.receive_port))
                receive_socket.listen(1)
                print(f"Receiving server listening on port {self.receive_port}")

                send_socket.bind(('0.0.0.0', self.send_port))
                send_socket.listen(1)
                print(f"Sending server listening on port {self.send_port}")

                send_obstacle_socket.bind(('0.0.0.0', self.send_obstacle_port))
                send_obstacle_socket.listen(1)
                print(f"Sending obstacle server listening on port {self.send_obstacle_port}")

                while True:
                    try:
                        receive_client, _ = receive_socket.accept()
                        print("Accepted receive connection")
                        break  # 성공적으로 연결되면 루프 탈출
                    except socket.timeout:
                        print("Waiting for receive connection...")
                        continue  # 타임아웃 발생 시 다시 시도

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
                send_thread = threading.Thread(target=self.send_messages, args=(self.send_data, send_client))
                send_obstacle_thread = threading.Thread(target=self.send_obstacle_messages, args=(send_obstacle_client,))

                receive_thread.start()
                send_thread.start()
                send_obstacle_thread.start()
                
                receive_thread.join()
                send_thread.join()
                send_obstacle_thread.join()
            
            time.sleep(0.1)
                
if __name__ == "__main__":
    jetson_socket = Server_pc()
    jetson_socket_thread = threading.Thread(target=jetson_socket.run)
    jetson_socket_thread.start()
    
