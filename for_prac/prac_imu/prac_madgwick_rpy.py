import rospy
import numpy as np
from sensor_msgs.msg import Imu
import os
from geometry_msgs.msg import Vector3Stamped  # angular velocity는 Vector3Stamped 타입

# File to save the RPY data

# File to save the RPY data
rpy_data_file = "./tmp/rpy_data.csv"

# Ensure that the directory exists
os.makedirs(os.path.dirname(rpy_data_file), exist_ok=True)

def quaternion_to_euler(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

def log_rpy(source, roll, pitch):
    with open(rpy_data_file, "a") as file:
        file.write(f"{source},{roll},{pitch}\n")

def imu_callback(data):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    acc_x = data.linear_acceleration.x
    acc_y = data.linear_acceleration.y
        
    roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
    
    # rospy.loginfo(f"madgwick - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
    rospy.loginfo(f"madgwick - acc_x : {acc_x:.2f}, acc_y: {acc_y:.2f}")
    # log_rpy("madgwick", roll, pitch, yaw)
    log_rpy("madgwick", acc_x, acc_y)
    
def imu_raw_callback(data):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w

    acc_x = data.linear_acceleration.x
    acc_y = data.linear_acceleration.y
    roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
    
    # rospy.loginfo(f"raw - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
    # rospy.loginfo(f"raw - acc_x: {acc_x:.2f}, acc_y: {acc_y:.2f}")
    # log_rpy("raw", acc_x, acc_y)

def imu_free_acc_callback(data):
    # x = data.orientation.x
    # y = data.orientation.y
    # z = data.orientation.z
    # w = data.orientation.w

    acc_x = data.vector.x
    acc_y = data.vector.y
    # roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
    
    # rospy.loginfo(f"raw - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
    rospy.loginfo(f"raw - acc_x: {acc_x:.2f}, acc_y: {acc_y:.2f}")
    log_rpy("raw", acc_x, acc_y)
    
if __name__ == '__main__':
    rospy.init_node('imu_quaternion_to_euler')
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    # rospy.Subscriber('/imu/data_raw', Imu, imu_raw_callback)
    rospy.Subscriber('/filter/free_acceleration', Vector3Stamped, imu_free_acc_callback)

    # Clear file before writing new data
    with open(rpy_data_file, "w") as file:
        file.write("source,roll,pitch,yaw\n")

    rospy.spin()
