import os
from datetime import datetime

def create_experiment_folder():
    base_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_folder = os.path.join(os.getcwd(), base_time)
    os.makedirs(base_folder, exist_ok=True)
    return base_folder

def log_message(message):
    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {message}")
