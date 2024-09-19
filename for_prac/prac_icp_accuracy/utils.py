import os
from datetime import datetime

def create_experiment_folder():
    # "log" 폴더 경로를 생성
    log_folder = os.path.join(os.getcwd(), "log")
    
    # 현재 시간을 기반으로 한 폴더 이름 생성
    base_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_folder = os.path.join(log_folder, base_time)
    
    # "log" 폴더가 없으면 생성
    os.makedirs(base_folder, exist_ok=True)
    
    return base_folder

def log_message(message):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {message}")