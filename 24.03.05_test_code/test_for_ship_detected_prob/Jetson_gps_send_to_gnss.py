import socket
import time

class UdpBroadcaster:
    def __init__(self, port):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting

    def send_data(self, data):
        self.sock.sendto(data.encode('utf-8'), ('<broadcast>', self.port))

    def close(self):
        self.sock.close()

if __name__ == "__main__":
    udp_broadcaster = UdpBroadcaster(10110)
    while True:
        time.sleep(1)
        udp_broadcaster.send_data("GPRMC,215235.670,A,3735.0064,N,12701.6746,E,0.000000,,060905,,*12")
        # print("sent")