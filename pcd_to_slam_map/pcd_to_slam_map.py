# Open3D 라이브러리를 사용하여 여러 PCD 파일을 읽고 통합하는 Python 스크립트 예시입니다.
# 이 스크립트는 각 PCD 파일을 순차적으로 로드하고, 이들을 하나의 큰 포인트 클라우드로 통합합니다.

import open3d as o3d
import numpy as np
import os

def load_and_combine_pcds(pcd_folder):
    # 폴더 내의 모든 PCD 파일 목록을 가져옵니다.
    pcd_files = [file for file in os.listdir(pcd_folder) if file.endswith('.pcd')]
    pcd_files.sort()  # 파일명에 따라 정렬
    
    combined_pcd = o3d.geometry.PointCloud()
    
    for pcd_file in pcd_files:
        # 각 PCD 파일을 로드합니다.
        pcd_path = os.path.join(pcd_folder, pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_path)
        
        # 로드된 PCD를 하나의 큰 포인트 클라우드로 통합합니다.
        combined_pcd += pcd
    
    # 최종 통합된 포인트 클라우드를 반환합니다.
    return combined_pcd

# PCD 파일이 저장된 폴더 경로를 지정하세요.
# pcd_folder = "~/Desktop/pcds/" ### python not allowed "~"

# PCD 파일이 저장된 폴더 경로를 지정하세요. ~를 사용하여 홈 디렉토리를 참조합니다.
pcd_folder = "~/Desktop/pcds/"
# ~를 실제 경로로 확장합니다.
pcd_folder = os.path.expanduser(pcd_folder)

# PCD 파일 로드 및 통합
combined_pcd = load_and_combine_pcds(pcd_folder)

# 선택적: 통합된 포인트 클라우드를 저장합니다.
o3d.io.write_point_cloud("combined_pcd.ply", combined_pcd)

# 선택적: 통합된 포인트 클라우드를 시각화합니다.
o3d.visualization.draw_geometries([combined_pcd])