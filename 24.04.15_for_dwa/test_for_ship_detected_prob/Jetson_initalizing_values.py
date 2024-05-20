import rospy
import std_msgs.msg
import time

def initialize_variables(self):
    self.end = 0
    self.flag_exit = False
    self.distance_to_target = 0
    self.distance_to_waypoint = 0

    self.isready = False
    self.isdriving = False
    self.isfirst = True
    # enddriving="0"
    self.driveindex = 0

    self.flag_avoidance = False
    self.message = None

    self.serial_gnss = None
    self.serial_nucleo = None

    self.serial_nucleo_cpy = None
    
    self.prev_pc_command = {'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF"}
    self.prev_pc_coeff = {"coeff_kf" : 3.5, "coeff_kd" : 0.6, "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : 0.2}
    self.coeff_vff_repulsive_force = 0.2

    self.flag_autodrive = False
    
    rospy.init_node("pointcloud_listener_and_publisher", anonymous=True) ### blocks if roslaunch not executed
    
    self.throttle_component = None
    self.roll_component = None
    
    self.autodrive_output_flag = False
    t = time.localtime()
    self.log_time = time.strftime("%H:%M:%S", t)

    self.last_received_time = None
    self.goal_subscribe_timeout = 3  # 10초 동안 업데이트가 없으면 경고
    self.flag_waypoint_publishing = False
    
    self.flag_stop_update_waypoint = False
