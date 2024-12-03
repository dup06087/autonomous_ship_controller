import rospy
from sensor_msgs.msg import Imu
import re
import json

# ROS 초기화
rospy.init_node('imu_data_publisher', anonymous=True)
imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

# IMU 데이터 메시지 생성
def create_imu_message(data):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "base_link"  # 원하는 좌표계 이름으로 수정 가능

    # 원하는 데이터 매핑
    imu_msg.orientation.x = 0.0  # 오리엔테이션 정보는 로그에서 추출하지 않아도 됩니다.
    imu_msg.orientation.y = 0.0
    imu_msg.orientation.z = 0.0
    imu_msg.orientation.w = 1.0  # 단위 쿼터니언
    imu_msg.linear_acceleration.x = 0
    imu_msg.linear_acceleration.y = 0
    imu_msg.linear_acceleration.z = 0.0  # z 축 가속도 값은 필요에 따라 설정
    imu_msg.angular_velocity.x = 0.0  # 각속도 값
    imu_msg.angular_velocity.y = 0.0
    imu_msg.angular_velocity.z = data['rotational_velocity'] * (3.14159 / 180.0)  # radian으로 변환
    print("imu_msg : ", imu_msg)
    return imu_msg

# 로그 파일 처리 및 IMU 데이터 퍼블리시
with open('log_current_value.txt', 'r') as f:
    for line in f:
        match = re.match(r"(\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d)\s+:\s+(.*)", line)
        if match:
            data = eval(match.group(2))  # JSON 데이터를 파싱하여 딕셔너리로 변환
            
            # IMU 메시지 생성
            imu_msg = create_imu_message(data)

            # IMU 데이터 퍼블리시
            imu_pub.publish(imu_msg)
            rospy.sleep(0.1)  # 주기 조정
