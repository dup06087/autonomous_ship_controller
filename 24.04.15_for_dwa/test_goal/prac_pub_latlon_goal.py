import rospy
from math import radians, cos, sin, pi
from geometry_msgs.msg import PoseStamped, Quaternion

def heading_to_quaternion(heading):
    heading_rad = radians(heading)
    w = cos(heading_rad / 2.0)
    x = 0.0
    y = 0.0
    z = sin(heading_rad / 2.0)
    return Quaternion(x=x, y=y, z=z, w=w)

def publish_nav_goal(start_lat, start_lon, radius, frame_id='map'):
    rospy.init_node('nav_goal_publisher')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz

    angle = 0  # 시작 각도
    while not rospy.is_shutdown():
        # 위도와 경도를 원형 경로로 계산
        delta_lat = radius * cos(radians(angle)) 
        delta_lon = radius * sin(radians(angle))
        
        current_lat = start_lat + delta_lat
        current_lon = start_lon + delta_lon

        # 헤딩을 쿼터니언으로 변환
        quaternion = heading_to_quaternion(angle)

        # PoseStamped 메시지 생성
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = current_lon  # 경도
        goal.pose.position.y = current_lat  # 위도
        goal.pose.orientation = quaternion

        # 목적지 발행
        pub.publish(goal)
        rate.sleep()

        angle = (angle + 1) % 360  # 각도 업데이트

if __name__ == '__main__':
    try:
        # 초기 위치, 원의 반경(미터 단위로 아주 작은 값)
        # publish_nav_goal(34.123456, -117.123456, 0.00000001)
        publish_nav_goal(0, 0, 10)
    except rospy.ROSInterruptException:
        pass
