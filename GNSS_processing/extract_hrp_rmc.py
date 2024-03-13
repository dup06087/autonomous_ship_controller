# NMEA 파일 경로
nmea_file_path = 'log_for_test.nma'  # 여기서는 예시 경로입니다. 실제 경로로 변경해야 합니다.

# 필터링된 데이터를 저장할 파일 경로
output_file_path = 'extracted_hrp_rmc.txt'

# 추출할 메시지 타입
message_types = ["$GNRMC", "$PSSN,HRP"]

# 파일 읽기 및 필터링
with open(nmea_file_path, 'r') as nmea_file, open(output_file_path, 'w') as output_file:
    for line in nmea_file:
        if line.startswith(tuple(message_types)):
            output_file.write(line)

# 파일이 저장된 경로를 출력
print(f'Filtered data saved to: {output_file_path}')
