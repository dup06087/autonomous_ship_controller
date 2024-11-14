import json
import math

# 파일에서 데이터를 읽고 x, y, yaw 누적 값을 계산하여 새 파일에 저장하는 함수
def accumulate_xy_yaw_with_heading(input_file_path, output_file_path):
    total_x = 0.0  # x 누적 합
    total_y = 0.0  # y 누적 합
    total_yaw = 0.0  # yaw 누적 합

    with open(input_file_path, 'r') as input_file, open(output_file_path, 'w') as output_file:
        for line in input_file:
            data = json.loads(line)
            
            # yaw를 누적하여 현재 heading 계산
            total_yaw += data['heading_change_gnss']
            yaw_rad = -math.radians(total_yaw)  # 라디안으로 변환
            
            # 현재 heading 방향을 고려하여 x, y 변화량을 누적
            dx = data['x_change_gnss'] * math.cos(yaw_rad) - data['y_change_gnss'] * math.sin(yaw_rad)
            dy = data['x_change_gnss'] * math.sin(yaw_rad) + data['y_change_gnss'] * math.cos(yaw_rad)
            total_x += dx
            total_y += dy
            
            # 누적 결과를 JSON 형식으로 저장
            result = {
                "x": total_x,
                "y": total_y,
                "yaw": total_yaw
            }
            output_file.write(json.dumps(result) + '\n')  # 한 줄씩 JSON으로 파일에 기록

    print(f"Data saved to {output_file_path}")

# 실행 예시
input_file_path = './Init_guess_I_offline/icp_gnss_results_gicp_round1.txt'  # 원본 JSON 파일 경로
output_file_path = 'accumulated_xy_yaw_with_heading_round1.txt'  # 결과 저장할 파일 경로
accumulate_xy_yaw_with_heading(input_file_path, output_file_path)
