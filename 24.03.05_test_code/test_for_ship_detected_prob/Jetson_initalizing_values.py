import rospy
import std_msgs.msg


def initialize_variables(self):
    self.end = 0
    self.flag_exit = False
    self.distance_to_target = 0

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
    
    
    