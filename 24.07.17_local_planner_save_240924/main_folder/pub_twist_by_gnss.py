import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import math

class VelocityPublisher:
    def __init__(self, mother_instance):
        print("VelocityPublisher Publishing")
        self.mother_instance = mother_instance
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # rospy.init_node('velocity_publisher', anonymous=True)
        self.rate = rospy.Rate(5)  # 10 Hz
        self.previous_time = rospy.Time.now()

    def publish_velocity(self):
        while not rospy.is_shutdown():
            try:
                odom = Odometry()
         
                current_time = rospy.Time.now()
                    
                # Set Odometry message
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"
                
                # Set the position (dummy values for demonstration)
                odom.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
                
                # Set the velocity
                odom.child_frame_id = "base_link"
                odom.twist.twist = Twist()
                
                forward_velocity = self.calculate_forward_velocity()
                angular_velocity = self.mother_instance.current_value['rotational_velocity']
                
                if forward_velocity is None or angular_velocity is None:
                    odom.twist.twist.linear.x = 0
                    odom.twist.twist.angular.z = 0
                else:        
                    odom.twist.twist.linear.x = forward_velocity
                    odom.twist.twist.angular.z = -angular_velocity
                
                self.pub.publish(odom)
                self.rate.sleep()
                
            except Exception as e:
                print("Pub twsit error : ", e)
    
    def calculate_forward_velocity(self):
        cog = self.mother_instance.current_value['COG']
        heading = self.mother_instance.current_value['heading']
        velocity = self.mother_instance.current_value['velocity']
        if cog is not None and heading is not None and velocity is not None:
            # 각도 차이 계산 (단위: degrees)
            delta_theta = abs(cog - heading)

            # 각도 차이를 radians로 변환
            delta_theta_rad = math.radians(delta_theta)

            # Forward Velocity 계산
            forward_velocity = round(velocity * math.cos(delta_theta_rad), 2)
        else:
            forward_velocity = 0
            
        self.mother_instance.current_value['forward_velocity'] = forward_velocity
        
        return forward_velocity
    
if __name__ == '__main__':
    from main import boat
    try:
        velocity_publisher = VelocityPublisher(boat)
        velocity_publisher.publish_velocity()
    except rospy.ROSInterruptException:
        pass
