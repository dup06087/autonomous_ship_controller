# main.py

import rospy
from ekf import LiDARIMUEKF  # ekf.py에서 LiDARIMUEKF 클래스를 가져옴
from main_in_one_page import IMUCorrector, ICPHandler  # main_in_one_file.py에서 클래스 가져옴

def main():
    # ROS 노드 초기화
    rospy.init_node('lidar_imu_fusion_system')

    # ekf.py에서 LiDARIMUEKF 클래스 인스턴스 생성
    ekf_node = LiDARIMUEKF()

    # main_in_one_file.py에서 IMUCorrector 클래스와 ICPHandler 클래스 인스턴스 생성
    imu_corrector = IMUCorrector(ekf_node)  # IMUCorrector 인스턴스 생성
    experiment_folder = "./ekf_results"  # 결과 저장 폴더 경로 설정
    icp_handler = ICPHandler(imu_corrector, ekf_node, experiment_folder)  # ICPHandler 인스턴스 생성

    # ROS 스핀
    rospy.spin()

if __name__ == "__main__":
    main()
