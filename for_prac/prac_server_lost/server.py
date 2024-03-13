import socket
import threading
import time

class Server:
    def __init__(self, receive_port=5003, send_port=5004):
        self.receive_port = receive_port
        self.send_port = send_port
        self.receive_socket = None
        self.send_socket = None

    def receive_messages(self, receive_socket):
        while True:
            try:
                data = receive_socket.recv(1024)
                if not data:
                    break
                print(f"Received: {data.decode()}")
            except ConnectionResetError:
                print("Receive connection lost.")
                break

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
            except ConnectionResetError:
                print("Connection was reset by the server. Attempting to reconnect...")
                break

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as receive_socket, \
                socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_socket:
            receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            receive_socket.bind(('0.0.0.0', self.receive_port))
            receive_socket.listen(1)
            print(f"Receiving server listening on port {self.receive_port}")

            send_socket.bind(('0.0.0.0', self.send_port))
            send_socket.listen(1)
            print(f"Sending server listening on port {self.send_port}")

            while True:
                receive_client, _ = receive_socket.accept()
                print("Accepted receive connection")

                send_client, _ = send_socket.accept()
                print("Accepted send connection")

                receive_thread = threading.Thread(target=self.receive_messages, args=(receive_client,))
                send_thread = threading.Thread(target=self.send_messages, args=(send_client,))

                receive_thread.start()
                send_thread.start()

if __name__ == "__main__":
    server = Server()
    server.run()