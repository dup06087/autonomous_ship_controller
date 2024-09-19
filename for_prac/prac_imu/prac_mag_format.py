import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped

def mag_callback(data):
    mag_msg = MagneticField()
    mag_msg.magnetic_field = data.vector  # Convert the vector
    mag_pub.publish(mag_msg)
    print("callback")

if __name__ == '__main__':
    rospy.init_node('mag_converter')
    mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)
    rospy.Subscriber('/imu/mag_raw', Vector3Stamped, mag_callback)
    rospy.spin()