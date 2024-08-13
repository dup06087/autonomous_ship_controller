import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import math

class VelocityPublisher:
    def __init__(self, mother_instance):
        print("VelocityPublisher Publishing")
        self.mother_instance = mother_instance
        if self.mother_instance.current_value['heading'] is not None:
            self.previous_heading = self.mother_instance.current_value['heading']
        else:
            self.previous_heading = 0
            
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # rospy.init_node('velocity_publisher', anonymous=True)
        self.rate = rospy.Rate(5)  # 10 Hz
        self.previous_time = rospy.Time.now()

    def publish_velocity(self):
        while not rospy.is_shutdown():
            try:
                odom = Odometry()
                if self.mother_instance.current_value['velocity'] == None:
                    current_velocity = 0
                else:
                    current_velocity = self.mother_instance.current_value['velocity']
                if self.mother_instance.current_value['heading'] == None:
                    current_heading = 0    
                else:
                    current_heading = self.mother_instance.current_value['heading']
                    
                current_time = rospy.Time.now()
                
                # Calculate time difference
                time_diff = (current_time - self.previous_time).to_sec()
                
                if time_diff > 0:
                    # Calculate heading difference
                    heading_diff = (current_heading - self.previous_heading + 360) % 360
                    if heading_diff > 180:
                        heading_diff -= 360
                        
                    print("odom heading_diff : ", heading_diff)
                    heading_diff_rad = math.radians(heading_diff)
                    print("odom heading diff radian ", heading_diff_rad)
                    # Calculate angular velocity
                    angular_velocity = round(heading_diff_rad / 0.2, 2)
                    
                    # Set Odometry message
                    odom.header.stamp = current_time
                    odom.header.frame_id = "odom"
                    
                    # Set the position (dummy values for demonstration)
                    odom.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
                    
                    # Set the velocity
                    odom.child_frame_id = "base_link"
                    odom.twist.twist = Twist()
                    odom.twist.twist.linear.x = current_velocity
                    odom.twist.twist.angular.z = angular_velocity
                    
                    # Update previous values
                    self.previous_heading = current_heading
                    self.previous_time = current_time
                    
                    self.pub.publish(odom)
                    print("odom published velocity : ", odom.twist.twist.linear.x, odom.twist.twist.angular.z)
                
                self.rate.sleep()
            except Exception as e:
                print("Pub twsit error : ", e)
                
if __name__ == '__main__':
    from main import boat
    try:
        velocity_publisher = VelocityPublisher(boat)
        velocity_publisher.publish_velocity()
    except rospy.ROSInterruptException:
        pass
