import time

from haversine import haversine
import math
import numpy as np

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Twist

from nav_msgs.msg import Path
from geographiclib.geodesic import Geodesic
import os

# auto_drive is executing with thread
def auto_drive(self):
    print("in the auto driving")
    self.cnt_destination = 0
    counter_dead_autodrive = 0
    prev_destination_latitude = []
    destination_latitude = []
    prev_destination_longitude = []
    destination_longitude = []
    
    prev_time = 0
    flag_arrived = False
    # self.current_value['cnt_destination'] = self.cnt_destination

    # 웨이포인트를 저장할 변수 초기화
    self.waypoint_latitude = 0.0
    self.waypoint_longitude = 0.0

    # rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.update_local_waypoint)
    # rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.update_global_waypoint)
    # rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.update_global_waypoint)
    rospy.Subscriber("/cmd_vel", Twist, self.update_cmd_vel)
    rospy.Timer(rospy.Duration(self.cmd_vel_timeout), self.check_cmd_vel_timeout)

    print("globalplanner subscribing")
    rospy.Timer(rospy.Duration(1), self.check_global_waypoint_timeout)

    last_print_time = time.time()  # 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占?占시곤옙 占십깍옙화

    distance_x = None
    distance_y = None
    
    while True:  
        try:
            # TODO : after controller on > check to see if autodriving well
            
            # true/false, lat, log, dest_lat, dest_lon
                            
            t = time.localtime()
            current_time = time.strftime("%H:%M:%S", t)
            # print("hello0")
            if flag_arrived:
                self.current_value["dest_latitude"] = None
                self.current_value["dest_longitude"] = None
                self.current_value["mode_pc_command"] = "SELF"
                self.cnt_destination = 0
                self.current_value["cnt_destination"] = self.cnt_destination
                destination_latitude = []
                destination_longitude = []

                flag_arrived = False
                
            # print("hello1")
            try:
                flag_ready_to_auto_drive, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading = self.flag_check_for_autodrive()
                # print("hello2")
                """ simulation
                flag_ready_to_auto_drive = True
                current_latitude, current_longitude, destination_latitude, destination_longitude
                """
                current_latitude, current_longitude, destination_latitude, destination_longitude = self.current_value["latitude"], self.current_value["longitude"], self.current_value["dest_latitude"], self.current_value["dest_longitude"]
                # print("hello3")

                current_heading = self.current_value["heading"]
                # print("hello4")
                if destination_latitude != prev_destination_latitude or destination_longitude != prev_destination_longitude: # arrived, new_destination_arrived
                    self.cnt_destination = 0
                    self.current_value["cnt_destination"] = self.cnt_destination
                    prev_destination_latitude = destination_latitude
                    prev_destination_longitude = destination_longitude
                # print("hello5")
                # print("flag check : ", self.flag_check_for_autodrive())
                if not flag_ready_to_auto_drive: # not ready                   
                    # counter_dead_autodrive += 1
                    # if counter_dead_autodrive >= 5: # time sleep 0.2s * 5
                    self.current_value["pwml_auto"] = 1500
                    self.current_value["pwmr_auto"] = 1500
                    self.current_value["waypoint_lat_m"] = None
                    self.current_value["waypoint_lon_m"] = None
                    # print("hello6")
                    if self.current_value["mode_pc_command"] == "AUTO":
                        
                        file_path = os.path.join(self.log_folder_path, "log_flag_stop.txt")
                        with file_path as file:
                            file.write(f"{self.log_time} : {self.autodrive_output_flag}\n")
                    
                    # cnt_destination still alive
                    time_ = time.time()
                    if time_ - prev_time >= 3:
                        print("manual driving")
                        prev_time = time_
                    
                    time.sleep(0.2)
            except Exception as e:
                pass
                # print("error auto drive : ", e)
                
            else: ### ready to auto drive
                try:
                    # print("hello0")
                    self.current_value["arrived"] = False
                    counter_dead_autodrive = 0
                    base_lat = current_latitude
                    base_lon = current_longitude
                    # print("hello1")
                    # if self.current_value["waypoint_lat_m"] == None or self.current_value["waypoint_lon_m"] == None:    
                    #     self.current_value["pwml_auto"] = 1500
                    #     self.current_value["pwmr_auto"] = 1500
                    #     continue
                    # else:
                    #     distance_x = self.current_value["waypoint_lat_m"]
                    #     distance_y = self.current_value["waypoint_lon_m"]

                    # current_heading = self.current_value["heading"]
                    # try: 
                    #     target_lat, target_lon = convert_metric_to_latlon(base_lat, base_lon, distance_x, distance_y, current_heading) # target : 위경도
                    #     adjusted_target_lat, adjusted_target_lon = adjust_gps_position(target_lat, target_lon, current_heading)
                    # except Exception as e:
                    #     print("error 3 : ", e)
                    
                    try:
                        # self.current_value["waypoint_latitude"] = adjusted_target_lat
                        # self.current_value["waypoint_longitude"] = adjusted_target_lon
                        # calculate_pwm_auto(self, current_latitude, current_longitude, float(adjusted_target_lat), float(adjusted_target_lon), current_heading, self.current_value['coeff_kf'], self.current_value['coeff_kd'])
                        calculate_pwm_auto(self, current_latitude, current_longitude, 0, 0, current_heading, self.current_value['coeff_kv_p'], self.current_value['coeff_kv_i'])
                    except Exception as e:
                        print("error 4 : ", e)
                    t = time.localtime()    
                    log_time = time.strftime("%H:%M:%S", t)

                    try:
                        # with open('log_pwm.txt', 'a') as file:
                        #     file.write("{} : {},{}\n".format(log_time, self.current_value["pwml_auto"], self.current_value["pwmr_auto"]))
                        # print(f"value check : {current_latitude} {current_longitude} {self.current_value['dest_latitude'][self.cnt_destination]} {self.current_value['dest_longitude'][self.cnt_destination]} {self.cnt_destination}")
                        self.distance_to_target = haversine((current_latitude, current_longitude),
                                            (self.current_value['dest_latitude'][self.cnt_destination], self.current_value['dest_longitude'][self.cnt_destination]), unit='m')
                
                        self.current_value['distance'] = float(self.distance_to_target)
                    except Exception as e:
                        pass
                        # print("error 5 : ", e)
                        
                    ''' self.distance_to_target 값 none 아닌지 확인하는 코드 추가'''
                    if float(self.distance_to_target) <= 2:
                        # print("where is the prob2")
                        
                        self.cnt_destination += 1
                        self.current_value['cnt_destination'] = self.cnt_destination
                        
                        if self.cnt_destination >= len(self.current_value['dest_latitude']):
                            # print("where is the prob3")
                            self.cnt_destination = 0
                            self.current_value['cnt_destination'] = self.cnt_destination
                            
                            self.current_value['pwml_auto'] = 1500
                            self.current_value['pwmr_auto'] = 1500
                            self.distance_to_target = None
                            self.current_value['distance'] = None
                            
                            # below pc command
                            self.current_value['mode_pc_command'] = "SELF"
                            self.current_value["dest_latitude"] = None
                            self.current_value["dest_longitude"] = None
                            
                            self.current_value["waypoint_latitude"] = None
                            self.current_value["waypoint_longitude"] = None
                            
                            print("Arrived")
                            flag_ready_to_auto_drive = False
                            flag_arrived = True
                            self.current_value["arrived"] = True
                            continue 
                except Exception as e:
                    pass
                    # print('error here 2 : ', e)
                    
                current_time = time.time()
                if current_time - last_print_time >= 3: 
                    try:
                        print("auto driving well")
                        last_print_time = current_time  
                    except :
                        print("NOOOOOp")

            # print("auto drive end")
            time.sleep(0.2)

        except Exception as e:
            self.current_value['pwml_auto'] = 1500
            self.current_value['pwmr_auto'] = 1500
            print("auto driving error : ", e)
            time.sleep(0.2)


def convert_metric_to_latlon(base_lat, base_lon, distance_x, distance_y, current_heading):
    """
    base_lat, base_lon: 현재 로봇의 위경도
    distance_x, distance_y: 메트릭 단위의 상대적 거리 (미터)
    current_heading: 현재 로봇의 헤딩 (도)
    
    반환 값: (목표 위도, 목표 경도)
    """
    # 거리와 방위각으로 위경도를 계산하기 위해 Geodesic 라이브러리를 사용
    geod = Geodesic.WGS84

    # 현재 헤딩을 기반으로 X, Y 거리를 방위각으로 변환
    azimuth = (math.degrees(math.atan2(-distance_y, distance_x)) + current_heading) % 360
    
    # 거리를 계산 (피타고라스의 정리)
    distance = math.sqrt(distance_x**2 + distance_y**2)
    
    # base 위치에서 해당 방위각과 거리로 이동했을 때의 위치 계산
    result = geod.Direct(base_lat, base_lon, azimuth, distance)
    # 계산된 목표 위치의 위경도 반환
    return (result['lat2'], result['lon2'])

def compute_angular_velocity(self):
    dt = 0.2  # 고정된 시간 간격
    if self.prev_value['heading'] is not None and self.current_value['heading'] is not None:
        heading_diff = self.current_value["heading"] - self.prev_value["heading"]
        if heading_diff > 180:
            heading_diff -= 360
        elif heading_diff < -180:
            heading_diff += 360
        angular_velocity = heading_diff / dt
        angular_velocity = angular_velocity * math.pi / 180
        # print("angular velocity in compute_angular_velocity : ", angular_velocity)
        
    else:
        angular_velocity = 0
    return angular_velocity

# def compute_angular_velocity(self):
#     dt = 0.2  # 고정된 시간 간격
#     if self.prev_heading is not None:
#         heading_diff = self.current_value["heading"] - self.prev_heading
#         if heading_diff > 180:
#             heading_diff -= 360
#         elif heading_diff < -180:
#             heading_diff += 360
#         angular_velocity = heading_diff / dt
#     self.prev_heading = self.current_value["heading"]
#     self.prev_time = time.time()
#     return angular_velocity
    
def calculate_pwm_auto(self, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading, Kf=2.5, Kd=0.318, Ki=0.1, dt=0.2):
    try:
        
        v_measured = self.current_value["velocity"]
        omega_measured = -compute_angular_velocity(self) # deg
        # print("current v,w : ", v_measured, omega_measured) 
        print("desired v,w(rad) : ", self.linear_x, self.angular_z) # rad
        
        if self.linear_x == 0 and self.angular_z == 0:
            self.current_value["pwml_auto"] = 1500
            self.current_value["pwmr_auto"] = 1500
            return
        
        v_control, self.prev_error_v, self.integral_v = compute_pid(
            self.linear_x, v_measured, dt, self.current_value["coeff_kv_p"], self.current_value["coeff_kv_i"], self.current_value["coeff_kv_d"],
            self.prev_error_v, self.integral_v)
        omega_control, self.prev_error_omega, self.integral_omega = compute_pid(
            self.angular_z, omega_measured, dt, self.current_value["coeff_kw_p"], self.current_value["coeff_kw_i"], self.current_value["coeff_kw_d"],
            self.prev_error_omega, self.integral_omega)
        

        b = 0.295  # 바퀴 사이 거리
        k_L = 0.2217  # 왼쪽 바퀴 계수
        k_R = 0.2217  # 오른쪽 바퀴 계수
        C_d = 30  # 원하는 속도 계수
        C_tau = -0.8  # 원하는 각속도 계수
        
        # if self.angular_z >=0:
        PWM_left = (b * C_d * v_control + C_tau * omega_control) / (2 * b * k_L) # sign is changed because of diffrence between rviz and heading + direction
        PWM_right = (b * C_d * v_control - C_tau * omega_control) / (2 * b * k_R)
        # elif self.angular_z < 0:
        #     PWM_left = (b * C_d * v_control + C_tau * omega_control) / (2 * b * k_L) # sign is changed because of diffrence between rviz and heading + direction
        #     PWM_right = (b * C_d * v_control - C_tau * omega_control) / (2 * b * k_R)
        
        max_pwm = 500

        PWM_left_normalized = max(-max_pwm, min(max_pwm, PWM_left))
        PWM_right_normalized = max(-max_pwm, min(max_pwm, PWM_right))

        PWM_left_mapped = 1500 + (PWM_left_normalized / max_pwm) * 500
        PWM_right_mapped = 1500 + (PWM_right_normalized / max_pwm) * 500

        alpha = 0.75
        PWM_left_LPF = alpha * PWM_left_mapped + (1 - alpha) * self.prev_value["pwml_auto"]
        PWM_right_LPF = alpha * PWM_right_mapped + (1 - alpha) * self.prev_value["pwmr_auto"]
        
        self.current_value["pwml_auto"] = int(PWM_left_LPF)
        self.current_value["pwmr_auto"] = int(PWM_right_LPF) 
        
        # print("desired v,w : ", self.linear_x, self.angular_z, ", PWML : ", int(PWM_left_LPF), ", PMWR : ", int(PWM_right_LPF), ", Omega Control : ", omega_control, ", Velocity Control : ", v_control)


        # print("calcuated pwm auto : ", PWM_left_LPF, PWM_right_LPF)
    except Exception as e:
        print("error pwm : ", e)
        
def compute_pid(setpoint, measured_value, dt, Kp, Ki, Kd, prev_error, integral):
    error = setpoint - measured_value
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, error, integral
             
def adjust_gps_position(lat, lon, heading, offset=0.6):
    """
    Adjust the GPS position to account for an offset due to sensor placement.
    
    :param lat: Latitude of the robot's GPS position
    :param lon: Longitude of the robot's GPS position
    :param heading: Current heading of the robot in degrees
    :param offset: Distance in meters that the GPS is offset behind the sensor (default is 0.6 meters)
    :return: (adjusted_latitude, adjusted_longitude)
    """
    # Convert heading to radians
    heading_rad = np.radians(heading)
    
    # Calculate the offset vector components
    delta_north = offset * np.cos(heading_rad)  # north component
    delta_east = offset * np.sin(heading_rad)   # east component
    
    # Approximate conversion from degrees to meters at the equator
    meters_per_degree_lat = 111320
    meters_per_degree_lon = 111320 * np.cos(np.radians(lat))
    
    # Adjust latitude and longitude
    adjusted_lat = lat + (delta_north / meters_per_degree_lat)
    adjusted_lon = lon + (delta_east / meters_per_degree_lon)
    
    return (adjusted_lat, adjusted_lon)

def rotate_vector(vector, angle):
    """
    Rotate a vector by a given angle.

    :param vector: [x, y] vector
    :param angle: Angle in degrees
    :return: Rotated vector
    """
    rad = np.radians(angle)
    rotation_matrix = np.array([[np.cos(rad), -np.sin(rad)], 
                                [np.sin(rad), np.cos(rad)]])
    return np.dot(rotation_matrix, vector)

def calculate_destination_vector(self):
    # 현재 위치
    current_lat = self.current_value['latitude']
    current_lon = self.current_value['longitude']
    
    # 현재 위치의 heading (예시 값, 실제 heading 값을 사용해야 함)
    current_heading = self.current_value['heading']

    # 목적지
    dest_lat = self.current_value['dest_latitude'][self.current_value['cnt_destination']]
    dest_lon = self.current_value['dest_longitude'][self.current_value['cnt_destination']]

    # 위도와 경도 차이
    delta_lat = dest_lat - current_lat  # 도 단위
    delta_lon = dest_lon - current_lon  # 도 단위

    # 도를 미터로 변환
    delta_lat_meters = delta_lat * 111000  # 위도 차이를 미터로 변환
    delta_lon_meters = delta_lon * 111000 * np.cos(np.radians(current_lat))  # 경도 차이를 미터로 변환

    # 글로벌 벡터
    global_vector = np.array([delta_lon_meters, delta_lat_meters])

    # heading을 라디안으로 변환
    heading_radians = np.radians(current_heading)

    # 로컬 벡터로 변환 (2D 회전 행렬 사용)
    rotation_matrix = np.array([[np.cos(heading_radians), np.sin(heading_radians)],
                                [-np.sin(heading_radians), np.cos(heading_radians)]])
    local_vector = np.dot(rotation_matrix, global_vector)

    return local_vector
