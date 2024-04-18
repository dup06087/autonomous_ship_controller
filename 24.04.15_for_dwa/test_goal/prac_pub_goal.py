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

def publish_nav_goal(start_lat, start_lon, frame_id='map'):
    rospy.init_node('nav_goal_publisher')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    current_lat = start_lat
    current_lon = start_lon
    heading = 0

    while not rospy.is_shutdown():
        # 경도는 고정, 위도를 조금씩 변경
        delta = 0.000001  # 위도 변경값
        current_lat += delta

        if current_lat >= start_lat + 0.000360:  # 위도를 0.000360도 증가시켰다면 초기 위치로 리셋
            current_lat = start_lat

        # UTM 좌표로 변환
        x, y, _, _ = utm.from_latlon(current_lat, current_lon)
        
        # 헤딩을 쿼터니언으로 변환
        quaternion = heading_to_quaternion(heading)

        # PoseStamped 메시지 생성
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = quaternion

        # 목적지 발행
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_nav_goal(34.123456, -117.123456)  # 초기 위도와 경도
    except rospy.ROSInterruptException:
        pass
