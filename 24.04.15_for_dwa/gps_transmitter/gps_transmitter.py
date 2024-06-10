import serial
import time

# socat -d -d pty,raw,echo=0 pty,raw,echo=0


# 사용할 가상 시리얼 포트 지정
port = "/dev/pts/7"  # 실제 사용하는 포트 번호로 변경
baudrate = 115200

# 파일 경로 설정
# gps_data_file = './extracted_hrp_rmc.txt'  # python3로 실행시
# gps_data_file = './24.04.15_for_dwa/gps_transmitter/gps_data.txt' # sh 실행시
# gps_data_file = './24.04.15_for_dwa/gps_transmitter/test_stay_start_point.txt' # sh 실행시
# gps_data_file = './24.04.15_for_dwa/gps_transmitter/test_heading_changing.txt' # sh 실행시
gps_data_file = './24.04.15_for_dwa/gps_transmitter/test_pitch_-45.txt' # sh 실행시

# gps_data_file = './24.04.15_for_dwa/gps_transmitter/test_latitude_changing.txt' # sh 실행시


# 시리얼 포트 초기화
ser = serial.Serial(port, baudrate)

try:
    with open(gps_data_file, 'r') as file:
        while True:
            try:
                # 파일에서 두 줄씩 읽기```
                lines = [next(file).strip(), next(file).strip()]

                # 두 줄의 데이터를 시리얼 포트를 통해 송신
                for line in lines:
                    ser.write(line.encode() + b'\n')
                    print(f"Sent data: {line}")

                # 1초 대기
                time.sleep(0.2)

            except StopIteration:
                # 파일의 끝에 도달했으면, 파일 포인터를 다시 시작으로 이동
                file.seek(0)

except KeyboardInterrupt:
    ser.close()  # 프로그램 종료 시 포트 닫기