#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion

rospy.init_node('mpc_dummy_data')
path_pub = rospy.Publisher('/reference_path', Path, queue_size=1)
curv_pub = rospy.Publisher('/reference_curvature', Float64MultiArray, queue_size=1)
speed_pub = rospy.Publisher('/reference_speeds', Float64MultiArray, queue_size=1)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

rate = rospy.Rate(20)
path = Path()
path.header.frame_id = 'map'
for i in range(100):
    p = PoseStamped()
    p.pose.position.x = 10.0 + i * 0.1
    p.pose.position.y = 0.0
    p.pose.orientation = Quaternion(0, 0, 0, 1)
    path.poses.append(p)
curv = Float64MultiArray(data=[0.0] * 100)
speed = Float64MultiArray(data=[1.0] * 100)
odom = Odometry()
odom.pose.pose.position.x = 0.0
odom.pose.pose.position.y = 0.0
odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)

while not rospy.is_shutdown():
    path.header.stamp = rospy.Time.now()
    path_pub.publish(path)
    curv_pub.publish(curv)
    speed_pub.publish(speed)
    odom_pub.publish(odom)
    rate.sleep()
