import time
import haversine
import math

# auto_drive is executing with thread
def auto_drive(self):
    print("in the auto driving")
    self.flag_is_auto_driving = False
    self.cnt_destination = 0
    # self.current_value['cnt_destination'] = self.cnt_destination

    last_print_time = time.time()  # 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占?占시곤옙 占십깍옙화

    while True:  
        try:
            # true/false, lat, log, dest_lat, dest_lon
            flag_ready_to_auto_drive, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading = self.flag_ready_to_auto_driving()
            if not flag_ready_to_auto_drive: # not ready                   
                self.current_value["pwml_auto"] = 1500
                self.current_value["pwmr_auto"] = 1500
                self.flag_is_auto_driving = False
                # cnt_destination still alive
                pass

            else: ### ready to auto drive
                self.flag_is_auto_driving = True
                if self.flag_avoidance:
                    destination_latitude = float(self.current_value["waypoint_latitude"])
                    destination_longitude = float(self.current_value["waypoint_longitude"])
                else:
                    destination_latitude = float(self.current_value['dest_latitude'][self.cnt_destination])
                    destination_longitude = float(self.current_value['dest_longitude'][self.cnt_destination])

                self.calculate_pwm_auto(current_latitude,current_longitude,destination_latitude,destination_longitude,current_heading)

                if float(self.distance_to_target) <= 3:
                    self.cnt_destination += 1
                    self.current_value['cnt_destination'] = self.cnt_destination
                    
                    # if alive
                    if self.cnt_destination >= len(self.current_value['dest_latitude']):
                        self.flag_is_auto_driving = False
                        self.cnt_destination = 0
                        self.current_value['cnt_destination'] = self.cnt_destination
                        
                        self.current_value['pwml_auto'] = 1500
                        self.current_value['pwmr_auto'] = 1500
                        self.distance_to_target = None
                        self.current_value['distance'] = None
                        
                        # below pc command
                        # self.current_value['mode_pc_command'] = "SELF"
                        # self.current_value["dest_latitude"] = None
                        # self.current_value["dest_longitude"] = None

                        return print("Arrived")

                current_time = time.time()
                if current_time - last_print_time >= 1: 
                    try:
                        last_print_time = current_time  
                    except :
                        print("NOOOOOp")

            time.sleep(0.1)


        except Exception as e:
            self.current_value['pwml_auto'] = 1500
            self.current_value['pwmr_auto'] = 1500
            print("auto driving error : ", e)
            
def calculate_pwm_auto(self, current_latitude, current_longitude, destination_latitude, destination_longitude, current_heading):
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

    Kf = 2.5
    # Kd = 0.25 * 800 / (2 * math.pi * 100)
    Kd = 0.318

    Uf = Kf * throttle_component
    Uf = max(1550 - 1500, min(Uf, 1750 - 1500))

    Ud = Kd * roll_component
    max_diff = 800 * 0.125
    Ud = max(-max_diff, min(Ud, max_diff))

    PWM_right = 1500 + Uf - Ud
    PWM_left = 1500 + Uf + Ud

    self.current_value["pwml_auto"] = int(PWM_left)
    self.current_value["pwmr_auto"] = int(PWM_right)