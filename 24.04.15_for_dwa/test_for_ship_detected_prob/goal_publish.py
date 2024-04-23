import rospy
import threading
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import math

class NavigationController:
    def __init__(self, boat_instance):
        self.boat = boat_instance
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.goal_lat = 0.0
        self.goal_lon = 0.0
        self.heading = 0.0
        self.frame_id = 'map'
        
        # Set up the timer to clear costmaps independently of navigation commands
        rospy.Timer(rospy.Duration(5), self.clear_costmaps_callback)  # Clears costmaps every 5 seconds

    def publish_nav_goal(self):
        self.client.wait_for_server()
        
        while not rospy.is_shutdown():  # 외부 루프
            try:
                if self.boat.current_value['flag_autodrive']:
                    # print("goal publsihing")
                    try:
                        lat = self.boat.current_value['latitude']
                        lon = self.boat.current_value['longitude']
                        dest_lat = self.boat.current_value['dest_latitude'][self.boat.current_value["cnt_destination"]]
                        dest_lon = self.boat.current_value['dest_longitude'][self.boat.current_value["cnt_destination"]]
                        heading = self.boat.current_value['heading']
                    except Exception as e:
                        print("variable error : ", e)
                        
                        
                    try:
                        dx, dy = self.get_relative_position(lat, lon, dest_lat, dest_lon, heading)
                        quaternion = self.heading_to_quaternion(heading)
                        goal = PoseStamped()
                        goal.header.stamp = rospy.Time.now()
                        goal.header.frame_id = self.frame_id
                        goal.pose.position.x = dx
                        goal.pose.position.y = dy
                        goal.pose.orientation.x = quaternion[0]
                        goal.pose.orientation.y = quaternion[1]
                        goal.pose.orientation.z = quaternion[2]
                        goal.pose.orientation.w = quaternion[3]
                    except Exception as e:
                        print("pose error : ", e)
                        
                    self.pub_goal.publish(goal)
                    print("Goal published: Position - ({}, {}), Orientation - {}".format(dx, dy, quaternion))
                    rospy.sleep(1)
                    
                else:
                    print("navigation else")
                    self.cancel_all_goals()
                    rospy.sleep(1)  # 중지 상태에서는 1초마다 확인
                    
            except Exception as e:
                pass
                # print("pub nav goal thread error : ", e)
                
    def cancel_all_goals(self):
        cancel_msg = GoalID()
        self.client.cancel_goal()
        self.pub_cancel.publish(cancel_msg)
        print("All goals canceled.")

    def clear_costmaps_callback(self, event):
        try:
            self.clear_costmaps_service()
            rospy.loginfo("Costmaps cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def heading_to_quaternion(self, heading):
        cy = math.cos(heading * 0.5)
        sy = math.sin(heading * 0.5)
        return (0, 0, sy, cy)

    def get_relative_position(self, current_lat, current_lon, goal_lat, goal_lon, heading):
        R = 6371000  # 지구 반경 (미터 단위)
        phi1 = math.radians(current_lat)
        phi2 = math.radians(goal_lat)
        delta_phi = math.radians(goal_lat - current_lat)
        delta_lambda = math.radians(goal_lon - current_lon)

        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        bearing = math.atan2(y, x)

        adjusted_bearing = bearing - heading + math.pi/2  # 시계 방향으로 90도 더 회전
        dx = distance * math.sin(adjusted_bearing)
        dy = distance * math.cos(adjusted_bearing)

        return dx, dy



if __name__ == '__main__':
    boat_instance = None  # Replace with actual instance if necessary
    nav_controller = NavigationController(boat_instance)