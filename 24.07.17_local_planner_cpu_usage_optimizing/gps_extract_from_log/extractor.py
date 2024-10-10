def extract_and_save_data(input_file, output_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()
    
    # 각 줄에서 `$` 이후의 내용만 추출
    extracted_lines = [line[line.index('$'):] if '$' in line else '' for line in lines]
    
    # 결과를 새 파일에 저장
    with open(output_file, 'w') as file:
        file.writelines(extracted_lines)

# 함수 사용 예
input_file = '24.04.09_one_lap.txt'
output_file = 'extracted_output.txt'
extract_and_save_data(input_file, output_file)