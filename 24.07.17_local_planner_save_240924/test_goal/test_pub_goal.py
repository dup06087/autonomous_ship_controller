import rospy
import utm
from math import radians, cos, sin
from geometry_msgs.msg import PoseStamped, Quaternion

def heading_to_quaternion(heading):
    heading_rad = radians(heading)
    w = cos(heading_rad / 2.0)
    x = 0.0
    y = 0.0
    z = sin(heading_rad / 2.0)
    return Quaternion(x=x, y=y, z=z, w=w)

def get_relative_position(lat1, lon1, lat2, lon2):
    # 첫 번째 위치(로봇의 현재 위치)를 UTM으로 변환
    x1, y1, _, _ = utm.from_latlon(lat1, lon1)
    # 두 번째 위치(목적지)를 UTM으로 변환
    x2, y2, _, _ = utm.from_latlon(lat2, lon2)
    # 두 위치의 차이 계산
    dx = x2 - x1
    dy = y2 - y1
    return dx, dy

def publish_nav_goal(current_lat, current_lon, goal_lat, goal_lon, heading, frame_id='map'):
    rospy.init_node('nav_goal_publisher')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # 초당 1회 발행

    while not rospy.is_shutdown():
        # 현재 위치를 기준으로 목적지의 상대 위치 계산
        dx, dy = get_relative_position(current_lat, current_lon, goal_lat, goal_lon)

        # 헤딩을 쿼터니언으로 변환
        quaternion = heading_to_quaternion(heading)

        # PoseStamped 메시지 생성
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = dx
        goal.pose.position.y = dy
        goal.pose.orientation = quaternion

        # 목적지 발행
        pub.publish(goal)
        print("publishing")
        rate.sleep()

if __name__ == '__main__':
    try:
        # 현재 위치와 목적지 예시
        publish_nav_goal(34.134556, -117.5432144, 34.134568, -117.5432148, 270)
    except rospy.ROSInterruptException:
        pass
