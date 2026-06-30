#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion

rospy.init_node('mpc_dummy_curve')
path_pub = rospy.Publisher('/reference_path', Path, queue_size=1, latch=True)
curv_pub = rospy.Publisher('/reference_curvature', Float64MultiArray, queue_size=1, latch=True)
speed_pub = rospy.Publisher('/reference_speeds', Float64MultiArray, queue_size=1, latch=True)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1, latch=True)

rate = rospy.Rate(20)
path = Path()
path.header.frame_id = 'map'

R = 5.0           # 转弯半径
k = 1.0 / R       # 曲率
n = 100
ds = 0.1
ox, oy = 10.0 + R, R  # 圆心
for i in range(n):
    s = i * ds
    p = PoseStamped()
    p.pose.position.x = ox + R * math.sin(s / R)
    p.pose.position.y = oy - R * math.cos(s / R)
    yaw = s / R
    q = Quaternion(0, 0, math.sin(yaw/2), math.cos(yaw/2))
    p.pose.orientation = q
    path.poses.append(p)

curv = Float64MultiArray(data=[k] * n)
speed = Float64MultiArray(data=[1.0] * n)
odom = Odometry()
odom.pose.pose.position.x = 5.0   # 加上 spawn_x=10 后为 15.0
odom.pose.pose.position.y = 0.0
odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)

while not rospy.is_shutdown():
    path.header.stamp = rospy.Time.now()
    path_pub.publish(path)
    curv_pub.publish(curv)
    speed_pub.publish(speed)
    odom_pub.publish(odom)
    rate.sleep()
