import rosbag
import json

def save_kalman_filter_state_to_txt(bag_file, output_file):
    # rosbag 파일 열기
    bag = rosbag.Bag(bag_file)

    # 텍스트 파일로 저장
    with open(output_file, 'w') as outfile:
        # /kalman_filter/state 토픽에서 메시지 읽기
        for topic, msg, t in bag.read_messages(topics=['/kalman_filter/state']):
            # 상태 벡터에서 lat, lon, heading 추출
            state_data = {
                'lat': msg.data[0],
                'lon': msg.data[1],
                'heading': msg.data[2]
            }
            # JSON 형식으로 변환하여 파일에 저장
            json.dump(state_data, outfile)
            outfile.write('\n')  # 각 항목을 줄바꿈으로 구분

    # bag 파일 닫기
    bag.close()

# 사용 예시
bag_file = '../../../../2024-09-03-16-00-23.bag'  # 분석할 bag 파일 경로
output_file = 'kalman_filter_state.txt'  # 저장할 txt 파일 경로

save_kalman_filter_state_to_txt(bag_file, output_file)
