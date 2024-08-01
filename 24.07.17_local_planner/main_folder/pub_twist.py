import rospy
from geometry_msgs.msg import Twist
import math
import threading

class VelocityPublisher:
    def __init__(self, mother_instance):
        self.mother_instance = mother_instance
        self.previous_latitude = self.mother_instance.current_value['latitude']
        self.previous_longitude = self.mother_instance.current_value['longitude']
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rospy.init_node('velocity_publisher', anonymous=True)
        self.rate = rospy.Rate(5)  # 5 Hz
        self.previous_time = rospy.Time.now()

    def calculate_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        # Haversine formula to calculate distance between two lat/lon points
        R = 6371000  # Radius of the Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        
        # Bearing calculation
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        return distance, bearing

    def publish_velocity(self):
        while not rospy.is_shutdown():
            twist = Twist()
            
            current_latitude = self.mother_instance.current_value['latitude']
            current_longitude = self.mother_instance.current_value['longitude']
            current_velocity = self.mother_instance.current_value['velocity']
            current_heading = self.mother_instance.current_value['heading']
            current_time = rospy.Time.now()
            
            # Calculate time difference
            time_diff = (current_time - self.previous_time).to_sec()
            
            if time_diff > 0:
                # Calculate distance and bearing
                try:
                    distance, bearing = self.calculate_distance_and_bearing(
                        self.previous_latitude, self.previous_longitude,
                        current_latitude, current_longitude
                    )
                except Exception as e:
                    print("twsit error : ", e)
                    return
                
                # Calculate heading difference
                heading_diff = (bearing - current_heading + 360) % 360
                heading_diff_rad = math.radians(heading_diff)
                
                # Calculate velocity components in global frame
                global_x_velocity = current_velocity * math.cos(heading_diff_rad)
                global_y_velocity = current_velocity * math.sin(heading_diff_rad)
                
                # Set Twist message
                twist.linear.x = global_x_velocity
                twist.linear.y = global_y_velocity
                twist.linear.z = 0
                
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

                # Update previous values
                self.previous_latitude = current_latitude
                self.previous_longitude = current_longitude
                self.previous_time = current_time
                
                # self.pub.publish(twist)
                print("velocity : ", twist.linear.x, twist.linear.y)
            self.rate.sleep()

if __name__ == '__main__':
    from main import boat
    try:
        velocity_publisher = VelocityPublisher(boat)
        velocity_publisher.publish_velocity()
    except rospy.ROSInterruptException:
        pass
