import time

from haversine import haversine
import math
import numpy as np
import os

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from geographiclib.geodesic import Geodesic


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

    # 웨이포인트를 받기 위한 구독자 설정 (토픽 이름과 메시지 유형은 예시일 뿐 실제 프로젝트에서 확인 필요)
    # rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.update_local_waypoint)
    # rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.update_global_waypoint)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.update_global_waypoint)

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
            if flag_arrived:
                self.current_value["dest_latitude"] = None
                self.current_value["dest_longitude"] = None
                self.current_value["mode_pc_command"] = "SELF"
                self.cnt_destination = 0
                self.current_value["cnt_destination"] = self.cnt_destination
                destination_latitude = []
                destination_longitude = []
                self.integral_ud = 0
                self.prev_roll_component = None
                
                flag_arrived = False
            try:
                flag_ready_to_auto_drive, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading = self.flag_check_for_autodrive()

                """ simulation
                flag_ready_to_auto_drive = True
                current_latitude, current_longitude, destination_latitude, destination_longitude
                """
                current_latitude, current_longitude, destination_latitude, destination_longitude = self.current_value["latitude"], self.current_value["longitude"], self.current_value["dest_latitude"], self.current_value["dest_longitude"]
                current_heading = self.current_value["heading"]
                
                if destination_latitude != prev_destination_latitude or destination_longitude != prev_destination_longitude: # arrived, new_destination_arrived
                    self.integral_ud = 0
                    self.prev_roll_component = None
                    self.cnt_destination = 0
                    self.current_value["cnt_destination"] = self.cnt_destination
                    prev_destination_latitude = destination_latitude
                    prev_destination_longitude = destination_longitude
                
                # print("flag check : ", self.flag_check_for_autodrive())
                if not flag_ready_to_auto_drive: # not ready                   
                    # counter_dead_autodrive += 1
                    # if counter_dead_autodrive >= 5: # time sleep 0.2s * 5
                    self.current_value["pwml_auto"] = 1500
                    self.current_value["pwmr_auto"] = 1500
                    self.current_value["waypoint_lat_m"] = None
                    self.current_value["waypoint_lon_m"] = None
                    self.integral_ud = 0
                    self.prev_roll_component = None
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
                print("error here : ", e)
                
            else: ### ready to auto drive
                try:
                    self.current_value["arrived"] = False
                    counter_dead_autodrive = 0
                    base_lat = current_latitude
                    base_lon = current_longitude
                    if self.current_value["waypoint_lat_m"] == None or self.current_value["waypoint_lon_m"] == None:    
                        self.current_value["pwml_auto"] = 1500
                        self.current_value["pwmr_auto"] = 1500
                        continue
                    else:
                        distance_x = self.current_value["waypoint_lat_m"]
                        distance_y = self.current_value["waypoint_lon_m"]

                    current_heading = self.current_value["heading"]
                    try: 
                        target_lat, target_lon = convert_metric_to_latlon(base_lat, base_lon, distance_x, distance_y, current_heading) # target : 위경도
                        adjusted_target_lat, adjusted_target_lon = adjust_gps_position(target_lat, target_lon, current_heading)
                    except Exception as e:
                        print("error 3 : ", e)
                    
                    try:
                        self.current_value["waypoint_latitude"] = adjusted_target_lat
                        self.current_value["waypoint_longitude"] = adjusted_target_lon
                        calculate_pwm_auto(self, current_latitude, current_longitude, float(adjusted_target_lat), float(adjusted_target_lon), current_heading, self.current_value['coeff_kf'], self.current_value['coeff_kd'])
                    except Exception as e:
                        
                        print("error 4 : ", e)
                    t = time.localtime()    
                    log_time = time.strftime("%H:%M:%S", t)

                    try:
                        # with open('log_pwm.txt', 'a') as file:
                        #     file.write("{} : {},{}\n".format(log_time, self.current_value["pwml_auto"], self.current_value["pwmr_auto"]))
                        self.distance_to_target = haversine((current_latitude, current_longitude),
                                            (self.current_value['dest_latitude'][self.cnt_destination], self.current_value['dest_longitude'][self.cnt_destination]), unit='m')
                
                        self.current_value['distance'] = float(self.distance_to_target)
                    except Exception as e:
                        print("error 5 : ", e)
                        
                    ''' self.distance_to_target 값 none 아닌지 확인하는 코드 추가'''
                    if float(self.distance_to_target) <= 2:
                        # print("where is the prob2")
                        
                        self.cnt_destination += 1
                        self.current_value['cnt_destination'] = self.cnt_destination
                        self.integral_ud = 0
                        self.prev_roll_component = None
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
                    print('error here 2 : ', e)
                    
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


def thrust(pwm):
    if pwm < 1500:
        return (1500 - pwm) * (9.5 / 500) * -1  # 음의 추진력
    else:
        return (pwm - 1500) * (11.3 / 500)     # 양의 추진력

def solve(thrust_target, forward=True):
    if forward:
        return 1500 + thrust_target * (500 / 11.3)
    else:
        return 1500 - thrust_target * (500 / 9.5)
    
def calculate_pwm_auto(self, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading, Kf=35, Kp_side=150, Ki_side=1, Kd_side=10, dt=0.2):
    try:
        # 현재 헤딩 값 조정
        if current_heading > 180:
            current_heading -= 360

        # 목표 지점까지의 거리 계산
        self.distance_to_waypoint = haversine((current_latitude, current_longitude),
                                              (destination_latitude, destination_longitude), unit='m')

        # 목표 각도 계산
        target_angle = math.degrees(
            math.atan2(destination_longitude - current_longitude, destination_latitude - current_latitude))

        # 각도 차이 계산
        angle_diff = target_angle - current_heading
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # P항 계산 (전방)
        self.throttle_component = self.distance_to_waypoint * math.cos(math.radians(angle_diff))
        
        # roll_component 계산 (측면 오차 기반)
        self.roll_component = self.distance_to_waypoint * math.sin(math.radians(angle_diff))

        # 측면 방향 P항 계산
        Pd = Kp_side * self.roll_component

        # D항 계산을 위한 이전 roll_component 저장
        if self.prev_roll_component is None:
            self.prev_roll_component = self.roll_component  # 첫 실행 시 초기화
        d_roll_component = (self.roll_component - self.prev_roll_component) / dt  # 변화율 계산
        self.prev_roll_component = self.roll_component  # 현재 roll_component 업데이트

        # I항 계산
        Id = Ki_side * self.roll_component 
        self.integral_ud += Id * dt

        integral_max = 100  # I항 최대치 설정
        self.integral_ud = max(-integral_max, min(self.integral_ud, integral_max))
        Id = self.integral_ud
        # D항 계산
        Dd = + Kd_side * d_roll_component

        # 총 출력 계산
        Ud_total = Pd + Id + Dd 
        Uf = Kf * self.throttle_component
        Uf = min(Uf, 1750 - 1500)  # 최대 출력 제한

        if Ud_total > 0:
            PWM_right = 1500 + Uf - Ud_total
            PWM_right = max(1250, min(1750, PWM_right))

            thrust_right = thrust(PWM_right)
            thrust_origin = thrust(1500 + Uf)
            thrust_correction = thrust_origin - thrust_right
            thrust_left = thrust(1500 + Uf) + thrust_correction
            PWM_left = solve(thrust_left, forward=thrust_left >= 0)
        else:
            PWM_left = 1500 + Uf + Ud_total
            PWM_left = max(1250, min(1750, PWM_left))
            print("PWM left : ", PWM_left)
            thrust_left = thrust(PWM_left)
            thrust_origin = thrust(1500 + Uf)
            thrust_correction = thrust_origin - thrust_left
            thrust_right = thrust(1500 + Uf) + thrust_correction
            PWM_right = solve(thrust_right, forward=thrust_right >= 0)

        PWM_right = max(1250, min(1750, PWM_right))
        PWM_left = max(1250, min(1750, PWM_left))

        print("Ud_total : ", Ud_total, " self.integral_ud : ", self.integral_ud, " Dd : ", Dd, " Pd : ", Pd, " PWM_right : ", PWM_right, " PWM_left : ", PWM_left)


        self.current_value["pwml_auto"] = int(PWM_left)
        self.current_value["pwmr_auto"] = int(PWM_right)

    except Exception as e:
        print("error pwm : ", e)

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
