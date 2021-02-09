#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
import numpy as np


#Scan around the robot
def callback_Scan(data):
    global global_scan
    global_scan = np.array(data.ranges)

# callback of tje velocity
def callback_Velo(data):
    global global_velo
    global global_velo_linear
    global_velo= [data.linear.x, data.linear.y, data.linear.z]  
    global_velo_linear = np.array([data.linear.x,data.linear.y ,data.linear.z]) 
    #If velocity is more than max -> adjust it
    if((global_velo_linear > maxVelo).any()):
        print("Max speed exceeded", global_velo_linear)
        setMaxVelo(global_velo_linear)
        velo_move.publish(pub_velo)
    

#Any distance is beneath the min Distance = Stop everything
def emergencyStop():
    pub_velo.linear.x=0.0
    pub_velo.linear.y=0.0
    pub_velo.linear.z=0.0
    pub_velo.angular.x=0.0
    pub_velo.angular.y=0.0
    pub_velo.angular.z=0.0

#Function for setting velocity if max is reached
def setMaxVelo(global_velo_linear):
    if(global_velo_linear[0] > maxVelo ):
        pub_velo.linear.x = maxVelo
    if(global_velo_linear[1]  > maxVelo):
        pub_velo.linear.y = maxVelo
    if(global_velo_linear[2]  > maxVelo):
        pub_velo.linear.z= maxVelo

#Function to adjust speed ro distance
def adjustSpeed():
    print("Adjust velocity to distance")
    actualDist = np.amin(global_scan)-minDist
    rangeDist = maxDist-minDist # Allowed  Range
    #print("actualDist", actualDist)
    newVelo = maxVelo*(actualDist/rangeDist)#Calc new Velo
    #If any Velo is above newVelo --> adjust
    if(global_velo_linear[0]> newVelo ):
        pub_velo.linear.x = newVelo
    if(global_velo_linear[1]> newVelo):
        pub_velo.linear.y = newVelo
    if(global_velo_linear[2] > newVelo):
        pub_velo.linear.z= newVelo


def checkWall():
    #GlobalValues
    global pub_velo
    global maxVelo
    global minDist
    global global_velo
    global maxDist
    global global_scan
    global global_velo_linear
    global velo_move
    global_velo_linear=np.array([0.0,0.0,0.0])
    #InitValues
    pub_velo = Twist()
    pub_velo.linear.x=0.0
    pub_velo.linear.y=0.0
    pub_velo.linear.z=0.0
    pub_velo.angular.x=0.0
    pub_velo.angular.y=0.0
    pub_velo.angular.z=0.0
    maxVelo = rospy.get_param('/dontHitWallsNode/maxVelo', 0.5) #max velocity
    minDist = rospy.get_param('/dontHitWallsNode/minDist', 0.30) #min DIstance to object --> robot stopps if smaller
    maxDist = rospy.get_param('/dontHitWallsNode/maxDist', 2.0) #Distance at which the robot will get slower

    #Subscribers and Publisher
    scan = rospy.Subscriber('/scan', LaserScan, callback_Scan)
    #velo_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    velo_move = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
    velo = rospy.Subscriber('/pioneer/cmd_vel', Twist, callback_Velo)
    #velo = rospy.Subscriber('/input/cmd_vel', Twist, callback_Velo)

    rospy.init_node('checkWall', anonymous=True)
    rate = rospy.Rate(2)


    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)

    while (not rospy.is_shutdown()):
        subrate.sleep()
        #Check if robot is already to close
        print('Velocity', global_velo_linear)
        if((global_scan < minDist).any()):
            emergencyStop()
            print("Emergency Stop")
            velo_move.publish(pub_velo)
        #Adjust speed relative to distance
        elif((global_scan < maxDist).any()):
            adjustSpeed()
            velo_move.publish(pub_velo)

if __name__ == '__main__':
    checkWall()