import socket
import threading
import json
import time

class Server_pc:
    def __init__(self, boat_instance, receive_port=5003, send_port=5004, send_obstacle_port=5005):
        self.boat = boat_instance
        self.receive_port = receive_port
        self.send_port = send_port
        self.send_obstacle_port = send_obstacle_port
        self.send_data = {"example_key": "example_value"}
        self.send_obstacle_data = {"obstacle": "example_obstacle"}
        self.pc_command = {'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF"}
        self.flag_socket_pc = [False, False, False]


    def receive_messages(self, receive_socket):
        last_print_time = time.time()
        while True:
            try:
                data = receive_socket.recv(1024).strip()
                if not data:
                    self.flag_socket_pc[0] = False
                    break

                if data.decode() == "heartbeat":
                    continue  # 하트비트 메시지는 무시

                try:
                    received_dict = json.loads(data.decode('utf-8'))
                    self.pc_command.update(received_dict)
                    self.flag_socket_pc[0] = True
                    receive_socket.sendall("ack".encode())  # Acknowledgment 메시지 보내기

                except (json.JSONDecodeError, TypeError, ValueError):
                    current_time = time.time()
                    if current_time - last_print_time >= 3:
                        print("Invalid data format: ", data)
                        last_print_time = current_time

            except (ConnectionResetError, BrokenPipeError) as e:
                print("Receive connection lost.", e)
                self.flag_socket_pc[0] = False
                break
            time.sleep(0.05)
        
    def send_messages(self, send_data, send_socket):
        self.message_to_pc = {}
        while True:
            time.sleep(0.2)
            try:
                self.message_to_pc = json.dumps(self.boat.current_value)
                self.message_to_pc += '\n'
                send_socket.sendall(self.message_to_pc.encode())
                self.flag_socket_pc[1] = True
            except (BrokenPipeError, ConnectionResetError) as e:
                print("Send connection lost:", e)
                self.flag_socket_pc[1] = False
                break

    def send_obstacle_messages(self, send_socket):
        while True:
            time.sleep(0.2)
            try:
                message = json.dumps(self.boat.lidar_processor.bbox_lists)
                send_socket.send((message + '\n').encode())
                self.flag_socket_pc[2] = True
            except BrokenPipeError as e:
                print("Send obstacle data connection lost. Attempting to reconnect...", e)
                self.flag_socket_pc[2] = False
                break
            except ConnectionResetError:
                self.flag_socket_pc[2] = False
                print("Connection for obstacle data was reset by the server. Attempting to reconnect...")
                break
            except Exception as e:
                print("data error : ", e)

    def run(self):
        while True:
            print("regenerating")
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

                # 각 소켓에 대한 타임아웃 설정
                receive_socket.settimeout(5)
                send_socket.settimeout(5)
                send_obstacle_socket.settimeout(5)

                # 각 소켓에 대해 무한 대기 설정
                while True:
                    try:
                        receive_client, _ = receive_socket.accept()
                        print("Accepted receive connection")
                        break
                    except socket.timeout:
                        print("Waiting for receive connection...")
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
                send_thread = threading.Thread(target=self.send_messages, args=(self.send_data, send_client))
                send_obstacle_thread = threading.Thread(target=self.send_obstacle_messages, args=(send_obstacle_client,))

                receive_thread.start()
                send_thread.start()
                send_obstacle_thread.start()
                
                receive_thread.join()
                send_thread.join()
                send_obstacle_thread.join()

if __name__ == "__main__":
    from main import boat
    boat_ = boat()
    jetson_socket = Server_pc(boat_)
    jetson_socket_thread = threading.Thread(target=jetson_socket.run)
    jetson_socket_thread.start()
