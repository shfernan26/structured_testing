from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Header

import rospy

import time


NUM_POINTS_TO_PUBLISH = 100
 
cmd_vels = []


radar_objects = []

distances = []



for i in range(100):
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = i
    vel.linear.z = i
    cmd_vels.append(vel)

for i in range(120):
    odom = Odometry()
    h = Header()
    h.seq = 1
    odom.header = h
    radar_objects.append(odom)
    print(odom.header.seq)


for i in range(100):
    d = Float32()
    d.data = i/100.0
    distances.append(d)


rospy.init_node("Observer_Tester")

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
lead_vehicle_pub = rospy.Publisher('/lead_vechicle', Float32, queue_size=10)
radar_pub = rospy.Publisher('/radar_tracked_objects', Odometry, queue_size=10)

for i in range(100):
    vel_pub.publish(cmd_vels[i])
    lead_vehicle_pub.publish(distances[i])
    radar_pub.publish(radar_objects[i])
    time.sleep(0.05)

