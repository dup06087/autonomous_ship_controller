import rospy
import threading
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import math, time
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int32  # 토픽에서 Int32 메시지를 수신하기 위한 임포트

class NavigationController:
    def __init__(self, boat_instance):
        self.boat = boat_instance
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # self.costmap_subscriber = rospy.Subscriber(
        #     "/move_base/global_costmap/costmap", 
        #     OccupancyGrid, 
        #     self.costmap_callback
        # ) # no need
        
        self.costmap_updates_subscriber = rospy.Subscriber(
            "/move_base/global_costmap/costmap_updates", 
            OccupancyGridUpdate, 
            self.costmap_updates_callback
        )

        # /move_base/global_costmap/inflation_layer/inflated_costmap_value 토픽 구독
        self.costmap_value_subscriber = rospy.Subscriber(
            "/move_base/global_costmap/inflation_layer/inflated_costmap_value",
            Int32,
            self.costmap_value_callback
        )
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.goal_lat = 0.0
        self.goal_lon = 0.0
        self.heading = 0.0
        self.frame_id = 'map'
        
        self.costmap = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.costmap_value = None  # 수신된 costmap_value 저장 변수
 
        # Set up the timer to clear costmaps independently of navigation commands
        # rospy.Timer(rospy.Duration(5), self.clear_costmaps_callback)  # Clears costmaps every 5 seconds

    def costmap_value_callback(self, msg):
        self.costmap_value = msg.data
        self.boat.current_value['obstacle_cost'] = self.costmap_value
        # print("costmap value : ", self.costmap_value)
        # rospy.loginfo(f"Received costmap value: {self.costmap_value}")
        
    # def costmap_callback(self, data):
    #     self.costmap = data.data
    #     self.resolution = data.info.resolution
    #     self.origin_x = data.info.origin.position.x
    #     self.origin_y = data.info.origin.position.y
    #     # print("{}, {}, {}, {}".format(self.costmap, self.resolution, self.origin_x, self.origin_y))
        
    def publish_nav_goal(self):
        self.client.wait_for_server()
        time_prev = time.time()
        counter_false_autodrive = 0  # Counter for consecutive falses

        while not rospy.is_shutdown():  # 외부 루프
            try:
                if self.boat.current_value['flag_autodrive']:

                    counter_false_autodrive = 0  # Reset counter if autodrive is true

                    # print("goal publsihing")
                    try:
                        lat = self.boat.current_value['latitude']
                        lon = self.boat.current_value['longitude']
                        dest_lat = self.boat.current_value['dest_latitude'][self.boat.current_value["cnt_destination"]]
                        dest_lon = self.boat.current_value['dest_longitude'][self.boat.current_value["cnt_destination"]]
                        heading = self.boat.current_value['heading']
                    except Exception as e:
                        time_current = time.time()
                        if time_current - time_prev >= 5:
                            print("publishing goal error : ", e)
                            time_prev = time_current
                        rospy.sleep(1)
                        continue
                        
                    try:
                        dx, dy = self.get_relative_position(lat, lon, dest_lat, dest_lon, heading)
                        
                        quaternion = self.heading_to_quaternion(math.atan2(dy, dx))
                        # quaternion = self.heading_to_quaternion(0)
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
                        time_current = time.time()
                        if time_current - time_prev >= 5:
                            print("publishing goal destination error : ", e)
                            time_prev = time_current
                        rospy.sleep(1)
                        continue
                    
                    self.pub_goal.publish(goal)
                    # print("Goal published: Position - ({}, {}), Orientation - {}".format(dx, dy, quaternion))
                    rospy.sleep(0.2)
                    
                else:
                    # 목표지점까지 취소하는것 : 이것은 flag autodrive보다도 엄격히 적용
                    counter_false_autodrive += 1  # Increment counter if autodrive is false
                    if counter_false_autodrive > 6: # 3 second
                        self.cancel_all_goals()
                        self.boat.current_value['pwml_auto'] = 1500
                        self.boat.current_value['pwmr_auto'] = 1500
                    rospy.sleep(0.5)  # 중지 상태에서는 1초마다 확인
                    
            except Exception as e:
                pass
                # print("pub nav goal thread error : ", e)
                
    def cancel_all_goals(self):
        cancel_msg = GoalID()
        self.client.cancel_goal()
        self.pub_cancel.publish(cancel_msg)
        # print("All goals canceled.")

    # def clear_costmaps_callback(self, event):
    #     try:
    #         self.boat.flag_stop_update_waypoint = True
    #         self.clear_costmaps_service()
    #         rospy.loginfo("Costmaps cleared successfully(per second).")
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s", e)
    #     except Exception as e:
    #         print("clear costmap callback : ", e)
        
    def costmap_updates_callback(self, data):
        if self.boat.flag_stop_update_waypoint:
            self.boat.flag_stop_update_waypoint = False  # 데이터 수신 시 플래그 설정

        
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

        # heading이 degree 단위로 주어진다면, 이를 radians로 변환
        heading_rad = math.radians(heading)

        # heading을 고려한 베어링 조정
        adjusted_bearing = bearing - heading_rad + math.pi / 2

        dx = distance * math.sin(adjusted_bearing)
        dy = distance * math.cos(adjusted_bearing)

        return dx, dy

if __name__ == '__main__':
    boat_instance = None  # Replace with actual instance if necessary
    nav_controller = NavigationController(boat_instance)