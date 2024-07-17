import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import numpy as np

# 로그 파일을 읽고 데이터를 파싱하는 함수
def parse_log_file(file_path):
    timestamps = []
    x_positions = []
    y_positions = []
    z_positions = []
    headings = []

    with open(file_path, 'r') as file:
        lines = file.readlines()
        start_time = float(lines[0].strip().split(',')[0])
        end_time = float(lines[-1].strip().split(',')[0])

        for line in lines[:120]:
            parts = line.strip().split(',')
            if len(parts) == 5:
                timestamp = float(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                heading = float(parts[4])
                
                # Heading 값 조정
                if heading > 180:
                    heading -= 360

                # 타임스탬프를 시작 시간으로부터의 초 단위로 변환
                relative_time = (timestamp - start_time) / 1_000_000_000

                timestamps.append(relative_time)
                x_positions.append(x)
                y_positions.append(y)
                z_positions.append(z)
                headings.append(heading)

    return timestamps, x_positions, y_positions, z_positions, headings

# 그래프를 그리는 함수
def plot_data(timestamps, x_positions, y_positions, z_positions, headings):
    plt.figure(figsize=(12, 8))

    plt.plot(timestamps, x_positions, label='X Position (m)', color='r')
    plt.plot(timestamps, y_positions, label='Y Position (m)', color='g')
    plt.plot(timestamps, z_positions, label='Z Position (m)', color='b')
    plt.plot(timestamps, headings, label='Heading (deg)', color='y')

    plt.xlabel('Time (s)')
    plt.ylabel('Values')
    plt.title('Position and Heading Over Time')
    plt.legend()
    plt.grid(True)
    plt.xticks(rotation=45)
    
    plt.tight_layout()
    plt.show()

# 로그 파일 경로
log_file_path = 'position_log_about_1hour.txt'

# 로그 파일 파싱
timestamps, x_positions, y_positions, z_positions, headings = parse_log_file(log_file_path)

# 그래프 그리기
plot_data(timestamps, x_positions, y_positions, z_positions, headings)
