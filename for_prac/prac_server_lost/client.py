import socket
import threading
import time

class Client:
    def __init__(self, receive_port=5004, send_port=5003):
        self.receive_port = receive_port
        self.send_port = send_port
        self.receive_socket = None
        self.send_socket = None

    def send_messages(self, send_socket):
        while True:
            try:
                message = "Hello Server!"
                send_socket.send(message.encode())
                print(f"Sent: {message}")
                time.sleep(2)
            except BrokenPipeError:
                print("Send connection lost. Attempting to reconnect...")
                break
            except (ConnectionResetError, OSError):
                print("Connection was reset by the server. Attempting to reconnect...")
                break

    def receive_messages(self, receive_socket):
        while True:
            try:
                data = receive_socket.recv(1024)
                if not data:
                    break
                print(f"Received: {data.decode()}")
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("Receive connection lost.")
                break

    def run(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_socket, \
                        socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_socket:

                    send_socket.connect(('localhost', self.send_port))
                    print("Connected to the server for sending.")

                    receive_socket.connect(('localhost', self.receive_port))
                    print("Connected to the server for receiving.")

                    send_thread = threading.Thread(target=self.send_messages, args=(send_socket,))
                    receive_thread = threading.Thread(target=self.receive_messages, args=(receive_socket,))

                    send_thread.start()
                    receive_thread.start()

                    send_thread.join()
                    receive_thread.join()

            except (ConnectionRefusedError, ConnectionResetError, OSError) as e:
                print(f"Error: {e}. Server is unavailable. Retrying in 5 seconds...")
                time.sleep(5)

if __name__ == "__main__":
    client = Client()
    client.run()