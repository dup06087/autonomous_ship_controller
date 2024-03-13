from PyQt5.QtCore import QThread
import socket
import json
import time

class Client(QThread):
    def __init__(self, receive_port=5004, send_port=5003, receive_obstacle_port=5005):
        super(Client, self).__init__()
        self.receive_port = receive_port
        self.send_port = send_port
        self.receive_obstacle_port = receive_obstacle_port
        self.send_data = {"mode_pc_command": "SELF", "dest_latitude": None, "dest_longitude": None}
        self.data = {}
        self.obstacle_data = {}
        self.socket_status = False

    def run(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_obstacle_socket:
                    
                    send_socket.connect(('localhost', self.send_port))
                    receive_socket.connect(('localhost', self.receive_port))
                    receive_obstacle_socket.connect(('localhost', self.receive_obstacle_port))

                    while True:
                        self.handle_send_data(send_socket)
                        self.handle_receive_data(receive_socket)
                        self.handle_receive_obstacle_data(receive_obstacle_socket)
                        time.sleep(0.05)

            except (ConnectionRefusedError, ConnectionResetError, OSError) as e:
                print(f"Connection error: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def handle_send_data(self, send_socket):
        try:
            if self.validate_data(self.send_data):
                message = json.dumps(self.send_data) + '\n'
                send_socket.sendall(message.encode())
        except BrokenPipeError:
            print("Send connection lost. Attempting to reconnect...")
        except ConnectionResetError:
            print("Connection was reset by the server. Attempting to reconnect...")

    def handle_receive_data(self, receive_socket):
        try:
            data = receive_socket.recv(1024)
            if data:
                received_dict = json.loads(data.decode('utf-8'))
                self.data = received_dict
                self.socket_status = True
        except (ConnectionResetError, json.JSONDecodeError, TypeError, ValueError):
            print("Error in receiving data.")

    def handle_receive_obstacle_data(self, receive_socket):
        try:
            data = receive_socket.recv(1024)
            if data:
                obstacle_data = json.loads(data.decode())
                self.obstacle_data = obstacle_data
        except (ConnectionResetError, json.JSONDecodeError):
            print("Error in receiving obstacle data.")

    def validate_data(self, data):
        required_keys = ["mode_pc_command", "dest_latitude", "dest_longitude"]
        return all(key in data for key in required_keys)

if __name__ == "__main__":
    client = Client()
    client.start()
