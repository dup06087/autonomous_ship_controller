import socket
import select
import time
import json
import threading
import subprocess

class JetsonSocket:
    def __init__(self):
        self.get_value_from_pc = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", # pc get params
        }

        self.flag_pc_recv_alive = False
        self.flag_pc_send_alive = False

    def socket_pc_recv(self, client_socket='0.0.0.0', recv_port=5004):
        self.kill_process_using_port(recv_port)
        time.sleep(1)
        server_socket_pc_recv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = client_socket
        
        port = recv_port
        server_address = (host, port)
        server_socket_pc_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket_pc_recv.bind(server_address)
        server_socket_pc_recv.listen(1)
        server_socket_pc_recv.settimeout(2.0)
        
        print("listening recv")
        while True:
            try:
                
                client_socket, client_address = server_socket_pc_recv.accept() # if not connected error occured
                self.client_socket_for_status = client_socket
                print(f"Connected by {client_address}")
                last_print_time = time.time()
                while True:
                    data = client_socket.recv(1024).strip()


                    if not data:
                        break

                    try:
                        received_dict = json.loads(data.decode('utf-8'))
                        self.get_value_from_pc['mode_pc_command'] = received_dict['mode_pc_command']
                        self.get_value_from_pc['dest_latitude'] = received_dict['dest_latitude']
                        self.get_value_from_pc['dest_longitude'] = received_dict['dest_longitude']
                        self.flag_pc_recv_alive = True

                        print("PC >> Jetson", received_dict)  # 占쏙옙쨔占?占쌩곤옙

                    except (json.JSONDecodeError, TypeError, ValueError):
                        current_time = time.time()
                        if current_time - last_print_time >= 1:
                            try:
                                print("Waiting for destination")
                                last_print_time = current_time  
                            except:
                                print("NOOOOOp")
                        continue

                    time.sleep(0.1)
            except Exception as e:
                print(f"PC recv connection Error: {e}")
                self.get_value_from_pc['mode_pc_command'] = -1
                self.get_value_from_pc['dest_latitude'] = None
                self.get_value_from_pc['dest_longitude'] = None
                self.flag_pc_recv_alive = False


    ### receive from PC
    def socket_pc_send(self, send_data, client_socket='0.0.0.0', send_port=5003):
        self.kill_process_using_port(send_port)
        time.sleep(1)
        server_socket_pc_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = client_socket
        port = send_port
        server_address = (host, port)
        server_socket_pc_send.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket_pc_send.bind(server_address)
        server_socket_pc_send.listen(1)
        server_socket_pc_send.settimeout(2)
        print("listening send")

        while True:

            try:
                client_socket, client_address = server_socket_pc_send.accept()

                # print(f"Connected by {client_address}")

                while True:
                    ready_to_read, ready_to_write, _ = select.select([], [client_socket], [], 1)
                    if ready_to_write:
                        if isinstance(send_data, dict):
                            message = json.dumps(send_data)
                            message += '\n'  # 占쏙옙占쏙옙占쏙옙 占쌩곤옙
                            try:
                                client_socket.sendall(message.encode())
                                self.flag_pc_send_alive = True
                                print("sended well")

                            except OSError as e:
                                print("Error in sending message:", e)  # 占쏙옙占쏙옙 占쏙옙쨔占?占쌩곤옙
                                raise Exception("Connection with client has been closed.")
                        else:
                            print("current_value is not a dictionary.")

                    time.sleep(0.05)

            except Exception as e:
                print(f"PC send connection Error: {e}")
                self.flag_pc_send_alive = False

    def socket_pc_send_obstacle(self, send_data, client_socket='0.0.0.0', send_port=5005):
        self.kill_process_using_port(send_port)
        time.sleep(1)
        server_socket_pc_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = client_socket
        port = send_port
        server_address = (host, port)
        server_socket_pc_send.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket_pc_send.bind(server_address)
        server_socket_pc_send.listen(1)
        server_socket_pc_send.settimeout(2)
        print("listening send")

        while True:

            try:
                client_socket, client_address = server_socket_pc_send.accept()

                # print(f"Connected by {client_address}")

                while True:
                    print("obstacle raw data", send_data)
                    ready_to_read, ready_to_write, _ = select.select([], [client_socket], [], 1)
                    if ready_to_write:
                        if isinstance(send_data, list):
                            message = json.dumps(send_data)
                            message += '\n'  # 占쏙옙占쏙옙占쏙옙 占쌩곤옙
                            try:
                                client_socket.sendall(message.encode())
                                
                                # print("seded well")

                            except OSError as e:
                                print("Error in sending message:", e)  # 占쏙옙占쏙옙 占쏙옙쨔占?占쌩곤옙
                                raise Exception("Connection with client has been closed.")
                        else:
                            print("current_value is not a list.")

                    time.sleep(0.05)

            except Exception as e:
                print(f"PC send connection Error: {e}")
                self.flag_pc_send_alive = False

    def kill_process_using_port(self, port_number):
        try:
            # lsof 명령을 실행하여 특정 포트를 사용 중인 프로세스 ID를 찾습니다.
            result = subprocess.check_output(f"lsof -i :{port_number} | grep LISTEN | awk '{{print $2}}'", shell=True).decode().strip()
            if result:
                process_id = result.split('\n')[0]  # 첫 번째 프로세스 ID를 가져옵니다.
                # kill 명령을 실행하여 프로세스를 종료합니다.
                subprocess.run(f"kill -9 {process_id}", shell=True)
                # print(f"Killed process {process_id} using port {port_number}")
            else:
                print(f"No process using port {port_number}")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    jetson_socket = JetsonSocket()
    recv_thread = threading.Thread(target=jetson_socket.socket_pc_recv)
    send_thread = threading.Thread(target=jetson_socket.socket_pc_send, args=({"a" : 1},))

    recv_thread.start()
    send_thread.start()
