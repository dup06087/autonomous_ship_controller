#!/usr/bin/env python
import rospy
from datetime import datetime, timedelta
import serial
import threading
import time  # time 모듈 추가

gps_data_file = "./24.04.09_one_lap.txt"

# 시리얼 포트 설정
port = "/dev/pts/2"  # 실제 사용하는 포트 번호로 변경해주세요.
baudrate = 115200
ser = serial.Serial(port, baudrate)

# 마지막 처리 시간
last_processed_time = None
lock = threading.Lock()

def find_and_send_gnss_logs(log_file_path):
    """
    GNSS 로그 파일에서 현재 시간과 일치하는 로그를 찾아 시리얼로 뿌립니다.
    """
    global last_processed_time
    current_time = datetime.now()
    current_time_str = current_time.strftime("%H%M%S")  # HHMMSS 형식

    with open(log_file_path, 'r') as file:
        for line in file:
            if current_time_str in line:
                try:
                    next_line = next(file).strip()  # 다음 라인 읽기
                    ser.write(next_line.encode() + b'\n')
                    print(f"Sent data: {next_line}")
                except StopIteration:
                    pass
                break

def periodic_task():
    global last_processed_time
    while True:
        with lock:
            current_time = datetime.now()
            # 1초 단위로 데이터 처리
            if not last_processed_time or (current_time - last_processed_time).seconds >= 1:
                find_and_send_gnss_logs(gps_data_file)
                last_processed_time = current_time
        time.sleep(1)  # 1초 대기

if __name__ == '__main__':
    try:
        thread = threading.Thread(target=periodic_task)
        thread.start()
        thread.join()
    except KeyboardInterrupt:
        ser.close()
