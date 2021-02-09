#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Point


#Scan around me
def callback_Scan(data):
    global global_scan
    global_scan = data.ranges

#Which Pos are we?
def callback_Pos(data):
    global global_pos
    global_pos = [data.x, data.y]
    

def mover():
    move = rospy.Publisher('move', String, queue_size=10)
    scan = rospy.Subscriber('/scan', LaserScan, callback_Scan)
    pos = rospy.Subscriber('/robot_pos', Point, callback_Pos)
    
    init_pos = rospy.ServiceProxy ('init_pos', InitPos)
    init_pos(2,0)

    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(2)

#S,W,N,E
    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        subrate.sleep()
        print("robot pos", global_pos)
        if(global_scan[2]>1.0):
            move.publish('N')
        elif(global_scan[3]>1.0):
            move.publish('E')
        else:
            print("Arrival")
            print("Done - Closing Node now")
            rospy.signal_shutdown("Done")

if __name__ == '__main__':
    mover()