import time

from haversine import haversine
import math
import numpy as np

# auto_drive is executing with thread
def auto_drive(self):
    print("in the auto driving")
    self.cnt_destination = 0
    prev_destination_latitude = []
    destination_latitude = []
    prev_destination_longitude = []
    destination_longitude = []
    
    prev_time = 0
    flag_arrived = False
    # self.current_value['cnt_destination'] = self.cnt_destination

    last_print_time = time.time()  # 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占?占시곤옙 占십깍옙화

    while True:  
        try:
            # TODO : after controller on > check to see if autodriving well
            
            # true/false, lat, log, dest_lat, dest_lon
            
            if flag_arrived:
                self.current_value["dest_latitude"] = None
                self.current_value["dest_longitude"] = None
                self.current_value["mode_pc_command"] = "SELF"
                self.cnt_destination = 0
                self.current_value["cnt_destination"] = self.cnt_destination
                destination_latitude = []
                destination_longitude = []

                flag_arrived = False
                
            flag_ready_to_auto_drive, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading = self.flag_check_for_autodrive()
            
            if destination_latitude != prev_destination_latitude or destination_longitude != prev_destination_longitude: # arrived, new_destination_arrived
                self.cnt_destination = 0
                self.current_value["cnt_destination"] = self.cnt_destination
                prev_destination_latitude = destination_latitude
                prev_destination_longitude = destination_longitude
            
            # print("flag check : ", self.flag_check_for_autodrive())
            if not flag_ready_to_auto_drive: # not ready                   
                self.current_value["pwml_auto"] = 1500
                self.current_value["pwmr_auto"] = 1500    
                # cnt_destination still alive
                time_ = time.time()
                if time_ - prev_time >= 3:
                    print("manual driving")
                    prev_time = time_

            else: ### ready to auto drive
                self.current_value["arrived"] = False
                
                
                """ comment out here to exclude vff """
                # try:
                #     calculate_vff_force(self, self.lidar_processor.bbox_lists)
                #     # print("vff force calculate done")
                # except Exception as e:
                #     print("calculate vff force error : ", e)
                # try:
                #     self.current_value["waypoint_latitude"], self.current_value["waypoint_longitude"] = convert_vff_to_gps_with_heading(self.vff_force, current_latitude, current_longitude, current_heading)
                #     self.flag_avoidance = True
                # except Exception as e:
                #     self.flag_avoidance = False
                #     print("no obstacle : ", e)
                ''' end '''
                
                # add flag avoidance
                self.flag_avoidance = False
                    
                if self.flag_avoidance:
                    destination_latitude = float(self.current_value["waypoint_latitude"])
                    destination_longitude = float(self.current_value["waypoint_longitude"])
                    calculate_pwm_auto(self, current_latitude,current_longitude,float(self.current_value['dest_latitude'][self.cnt_destination]),float(self.current_value['dest_longitude'][self.cnt_destination]),current_heading, Kf = self.current_value['coeff_kf'], Kd = self.current_value['coeff_kd'])

                else:
                    destination_latitude = float(self.current_value['dest_latitude'][self.cnt_destination])
                    destination_longitude = float(self.current_value['dest_longitude'][self.cnt_destination])
                    calculate_pwm_auto(self, current_latitude,current_longitude,float(self.current_value['dest_latitude'][self.cnt_destination]),float(self.current_value['dest_longitude'][self.cnt_destination]),current_heading, Kf = self.current_value['coeff_kf'], Kd = self.current_value['coeff_kd'])
                
                ''' self.distance_to_target 값 none 아닌지 확인하는 코드 추가'''
                if float(self.distance_to_target) <= 3:
                    # print("where is the prob2")
                    self.cnt_destination += 1
                    self.current_value['cnt_destination'] = self.cnt_destination
                    
                    # if alive
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

                        print("Arrived")
                        flag_ready_to_auto_drive = False
                        flag_arrived = True
                        self.current_value["arrived"] = True
                        continue 
                    
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

def calculate_pwm_auto(self, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading, Kf = 2.5, Kd = 0.318):
    try:
       
        if current_heading > 180:
            current_heading = current_heading - 360
        
        self.distance_to_target = haversine((current_latitude, current_longitude),
                                            (destination_latitude, destination_longitude), unit='m')
        self.current_value['distance'] = float(self.distance_to_target)

        target_angle = math.degrees(
            math.atan2(destination_longitude - current_longitude, destination_latitude - current_latitude))

        angle_diff = target_angle - current_heading
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        throttle_component = self.distance_to_target * math.cos(math.radians(angle_diff))
        roll_component = self.distance_to_target * math.sin(math.radians(angle_diff))

        # Kf = 2.5
        # # Kd = 0.25 * 800 / (2 * math.pi * 100)
        # Kd = 0.318

        Uf = Kf * throttle_component
        Uf = max(1550 - 1500, min(Uf, 1750 - 1500))

        Ud = Kd * roll_component
        max_diff = 800 * 0.125
        Ud = max(-max_diff, min(Ud, max_diff))

        PWM_right = 1500 + Uf - Ud
        PWM_left = 1500 + Uf + Ud

        self.current_value["pwml_auto"] = int(PWM_left)
        self.current_value["pwmr_auto"] = int(PWM_right)
        
    except Exception as e:
        print("error pwm : ", e)
        
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

def convert_vff_to_gps_with_heading(vff_force, current_latitude, current_longitude, heading):
    """
    Convert VFF force (in meters) to GPS coordinate changes, considering the heading.

    :param vff_force: [x, y] force vector where x is forward and y is left in meters
    :param current_latitude: Current latitude in degrees
    :param current_longitude: Current longitude in degrees
    :param heading: Current heading in degrees
    :return: New latitude and longitude
    """
    
    # print("vff value check : ", vff_force, current_latitude, current_longitude, heading)
    earth_radius = 6371000  # 지구의 반경 (미터 단위)

    # VFF 벡터를 선박의 heading에 따라 회전
    rotated_force = rotate_vector(vff_force, heading)

    # 위도 및 경도 변화 계산
    delta_latitude = rotated_force[1] / earth_radius * (180 / np.pi)  # 위도 변화 (도 단위)
    delta_longitude = rotated_force[0] / (earth_radius * np.cos(np.pi * current_latitude / 180)) * (180 / np.pi)  # 경도 변화 (도 단위)

    # 새로운 위도 및 경도
    vff_latitude = current_latitude + delta_latitude
    vff_longitude = current_longitude + delta_longitude
    
    return vff_latitude, vff_longitude
    
def calculate_vff_force(self, obstacles):
    """
    Calculate the VFF force for a given list of obstacles and destination.
    
    :param obstacles: List of obstacles where each obstacle is represented as [x, y, w, h]
    :return: The resulting VFF force as a numpy array
    """
    
    ship_position = np.array([self.current_value['latitude'], self.current_value['longitude']])
    ship_center = np.array([0, 0])  # 선박의 현재 위치
    destination_vector = calculate_destination_vector(self)

    # 척력 계산
    repulsive_forces = []
    
    for obs in obstacles:
        center_x = obs[0] + obs[2] / 2  # 장애물 중심 X 좌표
        center_y = obs[1] + obs[3] / 2  # 장애물 중심 Y 좌표
        center = np.array([center_x, center_y])  # 장애물 중심 좌표
        distance = np.linalg.norm(center - ship_center)
        direction = (center - ship_center) / distance
        force_magnitude = 1 / (distance * self.coeff_vff_repulsive_force) **2
        # print("repulsived check updated : ", self.coeff_vff_repulsive_force)
        repulsive_forces.append(-force_magnitude * direction)
    # 인력 계산
    distance_to_destination = np.linalg.norm(destination_vector)
    attraction_direction = destination_vector / distance_to_destination
    attraction_force = distance_to_destination  # 거리에 비례하는 인력
    # 척력과 인력의 결합
    total_repulsive_force = np.sum(repulsive_forces, axis=0)
    
    total_force = total_repulsive_force + attraction_force * attraction_direction
    self.vff_force = total_force.tolist()  ### final output >> subscribe and show in rviz
    
    # repulsive force test
    # self.vff_force = total_repulsive_force.tolist()
    # print("repulsive force : ", self.vff_force)
    # attract force test
    # total_force = attraction_force * attraction_direction
    # self.vff_force = total_force.tolist()


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
