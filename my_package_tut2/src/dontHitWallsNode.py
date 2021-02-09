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

# callback of the velocity
def callback_Velo(data):
    global velo
    velo = data #Twist
    

#Function to adjust speed ro distance
def adjustSpeed(velo_move):
    global velo
    global global_scan
    global minDist
    global maxDist
    global maxVelo
    myDistX = np.amin(global_scan[81:164])
    myDistYR = np.amin(global_scan[0:81])
    myDistYL = np.amin(global_scan[164:245])
    if(velo.linear.x > 0): #Is moving forward
        if(myDistX < minDist): #Stop
            velo.linear.x=0
        elif(myDistX > minDist and myDistX < maxDist): #Adjust Speed
            velo.linear.x=maxVelo*((myDistX-minDist)/(maxDist-minDist))
        else: 
            velo.linear.x=maxVelo

    if(velo.linear.y > 0): #Is moving left
        if(myDistYR < minDist): #Stop
            velo.linear.y=0
        elif(myDist > minDist and myDistYL < maxDist): #Adjust Speed
            velo.linear.y=maxVelo*((myDistYL-minDist)/(maxDist-minDist))
        else: 
            velo.linear.y=maxVelo

    if(velo.linear.y < 0): #Is moving right
        if(myDistYR < minDist): #Stop
            velo.linear.y=0
        elif(myDistYR > minDist and myDistYR < maxDist): #Adjust Speed
            velo.linear.y=-maxVelo*((myDistYR-minDist)/(maxDist-minDist))
        else: 
            velo.linear.y=-maxVelo 
    velo_move.publish(velo)

def checkWall():
    #GlobalValues
    global maxVelo
    global minDist
    global maxDist
    global velo
    velo = Twist()
    #InitValues
    maxVelo = rospy.get_param('/dontHitWallsNode/maxVelo', 0.5) #max velocity
    minDist = rospy.get_param('/dontHitWallsNode/minDist', 0.30) #min DIstance to object --> robot stopps if smaller
    maxDist = rospy.get_param('/dontHitWallsNode/maxDist', 2.0) #Distance at which the robot will get slower
    robot = rospy.get_param('/dontHitWallsNode/ROBOT', 'rto-1') #Env
    print("Environment ROBOT:", robot)
    #Subscribers and Publisher
    if(robot == 'rto-1'):
        velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    else:
        velo_move = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
    #velo_move = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
    velocity = rospy.Subscriber('/input/cmd_vel', Twist, callback_Velo)

    rospy.init_node('checkWall', anonymous=True)
    rate=30.0

    while (not rospy.is_shutdown()):
        #subrate.sleep()
        newScan = rospy.wait_for_message("scan", LaserScan)
        callback_Scan(newScan)
        adjustSpeed(velo_move)
        if rate:
               rospy.sleep(1/rate)

if __name__ == '__main__':
    checkWall()