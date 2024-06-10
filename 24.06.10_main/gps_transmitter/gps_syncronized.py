#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from datetime import datetime, timedelta
import serial
import threading
import sys  # sys 모듈 추가

gps_data_file = "./24.04.09_one_lap.txt"

# 시리얼 포트 설정
port = "/dev/pts/5"  # 실제 사용하는 포트 번호로 변경해주세요.
baudrate = 115200
ser = serial.Serial(port, baudrate)

# 콜백 실행 중 플래그 및 마지막 처리 초
callback_in_progress = True
last_processed_second = None
lock = threading.Lock()

def datetime_from_rospy_time(rospy_time):
    """
    rospy.Time 객체를 datetime 객체로 변환합니다.
    """
    epoch = datetime.utcfromtimestamp(0)  # UTC 에포크
    return epoch + timedelta(seconds=rospy_time.secs, microseconds=rospy_time.nsecs / 1000)

def find_and_send_gnss_logs(gnss_time, log_file_path):
    """
    GNSS 로그 파일에서 특정 시간과 일치하는 로그를 찾아 시리얼로 뿌립니다.
    """
    with open(log_file_path, 'r') as file:
        for line in file:
            if gnss_time in line:
                line = next(file).strip()
                ser.write(line.encode() + b'\n')
                print(f"Sent data: {line}")
                try:
                    second_line = next(file).strip()
                    ser.write(second_line.encode() + b'\n')
                    print(f"Sent data: {second_line}")
                except StopIteration:
                    pass
                break

def clock_callback(msg):
    global callback_in_progress, last_processed_second
    
    with lock:
        if callback_in_progress:
            return  # 이미 실행 중인 경우, 콜백 처리를 건너뜀
        callback_in_progress = True  # 콜백 실행 시작을 표시
    
    try:
        current_time = datetime_from_rospy_time(msg.clock)
        current_second = current_time.strftime("%H%M%S")  # HHMMSS 형식
        # 초가 변경되었는지 확인
        if current_second != last_processed_second:
            print("doing~")
            last_processed_second = current_second
            gnss_time_format = current_second  # 시간 형식에 맞춤
            find_and_send_gnss_logs(gnss_time_format, gps_data_file)
            
    finally:
        with lock:
            callback_in_progress = False  # 콜백 실행 완료를 표시

def listen_to_clock_and_send_gnss():
    rospy.init_node('gnss_serial_sender', anonymous=True)
    rospy.Subscriber("/clock", Clock, clock_callback)
    print("spinning")
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listen_to_clock_and_send_gnss()
    except rospy.ROSInterruptException:
        ser.close()