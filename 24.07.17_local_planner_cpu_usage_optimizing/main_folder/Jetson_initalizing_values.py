import rospy
import std_msgs.msg
import time

def initialize_variables(self):
    self.end = 0
    self.flag_exit = False
    self.distance_to_target = 0
    self.distance_to_waypoint = 0
    self.integral_angle_diff = 0  # 각도 차이의 적분 초기화

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
    self.prev_pc_coeff = {"coeff_kv_p" : 3.5, "coeff_kv_i" : 0.6, "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : 0}
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
    
    self.cnt_gnss_signal_error = 0
    
    self.flag_stop_update_waypoint = False

    self.current_gps_data = {
        'latitude': None,
        'longitude': None,
        'heading': None
    }

    self.integral_ud = 0
    self.prev_roll_component = None
    
    self.linear_x = 0
    self.angular_z = 0
    
    self.prev_error_v = 0
    self.integral_v = 0
    self.prev_error_omega = 0
    self.integral_omega = 0

    self.Kp_v = 1.0
    self.Ki_v = 0.0
    self.Kd_v = 0.00
    self.Kp_omega = 1.0
    self.Ki_omega = 0.0
    self.Kd_omega = 0.00
    
    self.prev_heading = None
    self.cmd_vel_timeout = 2.0  # 1 second timeout

    self.flag_icp_execute = False

    self.prev_value = {
            # dest_latitude, dest_longitude : list, connected with pc def start_driving
            'dest_latitude': None, 'dest_longitude': None, 'mode_pc_command': "SELF", 'com_status_send': False, 'com_status_recv': False, 
            "coeff_kv_p" : 3.5, "coeff_kv_i" : 0.6, "coeff_kv_d" : 3.5, "coeff_kw_p" : 0.6, "coeff_kw_i" : 3.5, "coeff_kw_d" : 0.6, 
            "voxel_size" : 0.05, "intensity" : 30, "dbscan_eps" : 0.1 , "dbscan_minpoints" : 5, "coeff_vff_repulsive_force" : 0,
            # pc get params
            'mode_chk': "SELF", 'pwml_chk': None, 'pwmr_chk': None, # nucleo get params
            'pwml_auto': None, 'pwmr_auto': None, 'pwml_sim': None, 'pwmr_sim': None, 'cnt_destination' : 0, 'distance': None, "waypoint_latitude" : None, "waypoint_longitude" : None, # auto drving
            "waypoint_lat_m" : None, "waypoint_lon_m" : None,
            # gnss get params below
            'velocity': None, 'heading': 0, 'roll': None, 'pitch': None, 'validity': None, 'time': None, 'IP': None, 'date': None,
            "longitude": 127.077618, "latitude": 37.633173, "obstacle_cost" : 0,
            "arrived" : False, "flag_autodrive" : False
            # gnss end
            }