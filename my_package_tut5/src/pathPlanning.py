#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.msg import Path
from geometry_msgs.msg import Point,PoseStamped, Quaternion
import numpy as np
import math


#Get the Map
def callback_map(data):
    global map
    global height
    global width
    height = data.info.height
    width = data.info.width
    map = np.array(data.data)

#get the Position
def callback_Pos(data):
    global pos
    pos = data
    #print("Position: \n", data)

#get the Goal discretize and check if it is valid 
def callback_goal(data):
    global map
    global height
    global goalDisc
    goal= np.array([data.pose.position.x,data.pose.position.y])
    bins = np.arange(1, 20, 1)
    goalDisc = np.digitize(goal,bins, right=False)
    x = goalDisc[0]
    y = goalDisc[1]
    if(map[y*height+x]== 100 or map[y*height+x]== -1):
        print('Goal is not valid')
    algo()

def makePath(resultList):
    global height
    global width
    global goalDisc
    global navigate
    global visualGoal
    global visualPath
    #Marker for Path
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "navigation"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.5
    marker.color.a = 1.0
    marker.color.b = 0.6
    marker.color.r = 0.0
    marker.color.g = 0.4
    marker.pose.position.x=0.0
    marker.pose.position.y=0.0
    marker.pose.position.z=0.0
    marker.pose.orientation = Quaternion(0, 0, 0, 1)
    # Marker for the goal
    goalMarker = Marker()
    goalMarker.header.frame_id = "map"
    goalMarker.ns = "navigation"
    goalMarker.id = 0
    goalMarker.type = Marker.CUBE
    goalMarker.action = Marker.ADD
    goalMarker.scale.x = 1
    goalMarker.scale.y = 1
    goalMarker.scale.z = 0.2
    goalMarker.color.a = 1.0
    goalMarker.color.r = 1.0
    goalMarker.color.g = 0.0
    goalMarker.color.b = 0.0
    goalMarker.pose.orientation = Quaternion(0, 0, 0, 1)
    goalMarker.pose.position.x = goalDisc[0]+0.5
    goalMarker.pose.position.y = goalDisc[1]+0.5
    #MakePath
    path = Path()
    path.header.frame_id="map"
    myPoint = resultList[-1] #Starting point (should be goal)
    while(int(myPoint[3])!=-100): #Until start is reached
        p = Point()
        pose=PoseStamped()
        pose.pose.position.x=myPoint[0]%width
        pose.pose.position.y=myPoint[0] // height
        pose.pose.position.z=0
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        p.x=myPoint[0]%width
        p.y=myPoint[0] // height
        p.z=0
        marker.points.append(p)
        path.poses.insert(0,pose)
        origin = myPoint[3]
        myPoint = resultList[resultList[:,0]==origin][0] #Search for next Point 
    #AddStart 
    pose=PoseStamped()
    pose.pose.position.x=myPoint[0]%height
    pose.pose.position.y=myPoint[0] // height
    pose.pose.position.z=0
    path.poses.insert(0,pose)
    navigate.publish(path)
    visualGoal.publish(goalMarker)
    visualPath.publish(marker)

def insertInList(myList,checkedList, id, costMove,costMap,idBefore):
    #Is the next field already checked?
    if(checkedList.shape[1]>0):
        if(np.any(checkedList[:,0])==id):
            checkedIndex = np.where(checkedList[:,0]==id)[0]
            row=checkedList[checkedIndex,1]
            #Value in checked list is bigger
            if(row>costMove+1):
                checkedList=np.delete(checkedList,checkedIndex,0)#Remove from nextToChecked
                myList=np.append(myList, [[id,costMove+1,costMove+1+costMap[id], idBefore]], axis=0)
                return (myList,checkedList)
            else:
                return(myList,checkedList)
    #Is the field in the next possible values?
    if(np.any(myList[:, 0] == id)):
        rowIndex = np.where(myList[:,0]==id)[0]
        toCheck=myList[myList[:,0] == id]
        if(toCheck[:,2]>costMove+1+costMap[id]):
            myList[rowIndex,2]=costMove+1+costMap[id]
            myList[rowIndex,1]=costMove+1
            myList[rowIndex,3] = idBefore
    else:
        ar = np.array([id,costMove+1,costMove+1+costMap[id],idBefore])
        myList = np.append(myList, [ar], axis=0)
    return myList,checkedList

#real cost per movement = 1
#Estimated cost = distance between a and b
def algo():
    global pos #MyPosition
    global map #MyMap
    global height
    global width
    global goalDisc #My Discretized Goal
    costMap = np.zeros(map.shape[0]) # copy map for cost estimates
    for i in range(0,map.shape[0]):
        dist = math.sqrt((goalDisc[1]-math.floor(i/height))**2+(goalDisc[0]-(i%height))**2)
        if(map[i]!= 100 and map[i] != -1):
            costMap[i]=dist #get estimated cost and don't care about obstacles
    costMap[int(pos.y*height+pos.x)]=0 #Start has no costs
    nextToCheck = np.array([[int(pos.y*height+pos.x), 0,0, -100]]) #init nextToCheck id, cost move, cost all, id from before
    checked = np.array([[-1,-1,-1,-1]])#Already checked
    searching = True
    while(searching):
        #Get minimum from nextToCheck, are all 4 points existing? And Are they valid?Is the min our goal?
        row = np.argmin(nextToCheck[:,2], axis=0)
        i=int(nextToCheck[row][0]) #get row id
        moveCostOld=nextToCheck[row][1] 
        if(i==int(goalDisc[1]*height+goalDisc[0])):
            searching=False
            checked=np.append(checked,[nextToCheck[row]], axis=0)
        else:
            if((i+1)<map.shape[0]):
                if(map[i+1]!= 100 and map[i+1] != -1 and (i%width)<19):
                    j = np.where(nextToCheck[:,0]==i+1)[0]
                    nextToCheck, checked = insertInList(nextToCheck,checked, i+1,moveCostOld, costMap, i)
            if(i-1>=0):
                if(map[i-1]!= 100 and map[i-1] != -1 and (i%width)>0):
                    j = np.where(nextToCheck[:,0]==i-1)[0]
                    nextToCheck, checked = insertInList(nextToCheck,checked, i-1,moveCostOld, costMap, i)
            if(i+10<map.shape[0]):
                if(map[i+20]!= 100 and map[i+20] != -1 and (i//height)<19):
                    j = np.where(nextToCheck[:,0]==i+20)[0]
                    nextToCheck, checked = insertInList(nextToCheck,checked, i+20,moveCostOld, costMap, i)
            if(i-20>=0):
                if(map[i-20]!= 100 and map[i-20] != -1 and (i//height)>0):
                    j = np.where(nextToCheck[:,0]==i-20)[0]
                    nextToCheck, checked = insertInList(nextToCheck,checked, i-20,moveCostOld, costMap,i)
            checked = np.append(checked,[nextToCheck[row]], axis=0) #Add into checked
            nextToCheck = np.delete(nextToCheck,row,0)#Remove from nextToChecked
    makePath(checked)


def pathPlanning():
    global navigate
    global visualGoal
    global visualPath
    #Publisher
    navigate = rospy.Publisher('/global_path', Path, queue_size=10, latch=True)
    visualGoal = rospy.Publisher('/visualization/goal', Marker, queue_size=10)
    visualPath = rospy.Publisher('/visualization/plan', Marker, queue_size=10)
    #Subscriber
    robotPosition = rospy.Subscriber('/robot_pos', Point, callback_Pos)
    map = rospy.Subscriber('/map', OccupancyGrid, callback_map)
    goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)   

    rospy.init_node('pathPlanning', anonymous=True)
    rate = rospy.Rate(2)


    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)

    while (not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    pathPlanning()