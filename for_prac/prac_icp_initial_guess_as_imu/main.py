import rospy
import signal
import sys
import os
from datetime import datetime
from gnss_handler import GNSSHandler
from icp_handler import ICPHandler
from utils import create_experiment_folder, log_message

class ExperimentManager:
    def __init__(self):
        rospy.init_node("experiment_manager", anonymous=True)

        self.base_folder = create_experiment_folder()
        self.experiment_count = 0
        self.gnss_handler = None
        self.icp_handler = None
        self.current_experiment_folder = None
        self.setup_new_experiment()

    def setup_new_experiment(self):
        self.experiment_count += 1
        self.current_experiment_folder = self.base_folder
        os.makedirs(self.current_experiment_folder, exist_ok=True)
        
        # self.gnss_handler = GNSSHandler(self.current_experiment_folder)
        self.icp_handler = ICPHandler(self.gnss_handler, self.current_experiment_folder)

        log_message(f"Started experiment {self.experiment_count}")

    def start(self):
        rospy.spin()
        # rate = rospy.Rate(10)  # 10Hz 주기로 실행
        # while not rospy.is_shutdown():
        #     self.icp_handler.process_lidar()
        #     rate.sleep()

    def save_and_exit(self):
        # self.gnss_handler.save_data()
        self.icp_handler.save_data()
        # self.gnss_handler.close()
        log_message("Experiment data saved and program exiting.")
        sys.exit(0)

    def signal_handler(self, sig, frame):
        log_message("Interrupt signal received.")
        self.save_and_exit()

    def keyboard_input(self):
        while True:
            key = input().strip().lower()
            if key == 'q':
                log_message("Ending current experiment and starting a new one.")
                self.save_and_exit()
                self.setup_new_experiment()
                self.start()
            elif key == 'r':
                log_message("Saving current experiment and shutting down.")
                self.save_and_exit()

if __name__ == "__main__":
    manager = ExperimentManager()
    signal.signal(signal.SIGINT, manager.signal_handler)
    manager.start()
    manager.keyboard_input()
