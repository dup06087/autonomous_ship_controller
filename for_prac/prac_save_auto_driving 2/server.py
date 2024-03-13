import socket
import threading
import time
import json

class Server:
    def __init__(self, receive_port=5003, send_port=5004, send_obstacle_port=5005):
        self.receive_port = receive_port
        self.send_port = send_port
        self.send_obstacle_port = send_obstacle_port
        self.send_data = {"example_key": "example_value"}  # Example data to send
        self.send_obstacle_data = [{"obstacle": "example_obstacle"}]  # Example obstacle data
        self.get_value_from_pc = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", # pc get params
        }

    def receive_messages(self, receive_socket):
        last_print_time = time.time()
        while True:
            try:
                data = receive_socket.recv(1024).strip()
                if not data:
                    break
                try:
                    received_dict = json.loads(data.decode('utf-8'))
                    self.get_value_from_pc.update(received_dict)
                    print("PC >> Jetson", received_dict)
                except (json.JSONDecodeError, TypeError, ValueError):
                    current_time = time.time()
                    if current_time - last_print_time >= 1:
                        print("Waiting for destination")
                        last_print_time = current_time
            except ConnectionResetError:
                print("Receive connection lost.")
                break

    def send_messages(self, send_data, send_socket):
        while True:
            try:
                message = json.dumps(send_data)
                message += '\n'
                send_socket.sendall(message.encode())
                print("Sent:", message)
                time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError) as e:
                print("Send connection lost:", e)
                self.flag_pc_send_alive = False
                break

    def send_obstacle_messages(self, send_socket):
        while True:
            try:
                message = json.dumps(self.send_obstacle_data)
                send_socket.send(message.encode())
                print(f"Sent obstacle data: {message}")
                time.sleep(0.1)
            except BrokenPipeError:
                print("Send obstacle data connection lost. Attempting to reconnect...")
                break
            except ConnectionResetError:
                print("Connection for obstacle data was reset by the server. Attempting to reconnect...")
                break

    def run(self):
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
                receive_client, _ = receive_socket.accept()
                print("Accepted receive connection")

                send_client, _ = send_socket.accept()
                print("Accepted send connection")

                send_obstacle_client, _ = send_obstacle_socket.accept()
                print("Accepted send obstacle connection")

                receive_thread = threading.Thread(target=self.receive_messages, args=(receive_client,))
                send_thread = threading.Thread(target=self.send_messages, args=(self.send_data, send_client))
                send_obstacle_thread = threading.Thread(target=self.send_obstacle_messages, args=(send_obstacle_client,))

                receive_thread.start()
                send_thread.start()
                send_obstacle_thread.start()

if __name__ == "__main__":
    jetson_socket = Server()
    jetson_socket_thread = threading.Thread(target=jetson_socket.run)
    jetson_socket_thread.start()