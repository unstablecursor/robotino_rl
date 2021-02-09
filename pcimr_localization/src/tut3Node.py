#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker
from threading import Lock

#same,left,right,backwards,stand Still
#global prob = [0.1,0.5,0.1,0.1,0.2]


#Map Callback
def callback_map(data):
    global map
    global mapGrid
    global FirstTime
    global navi 
    my_lock.acquire()
    #calculate first map
    if(FirstTime):
        mapGrid = OccupancyGrid()
        mapGrid.header.frame_id = 'map'
        mapGrid.header.seq = 0
        mapGrid.info.map_load_time = rospy.Time.now()
        mapGrid.info.resolution = 1
        mapGrid.info.height = 20
        mapGrid.info.width = 20
        map = np.copy(data.data).astype(np.float32)
        probs = map.shape[0]-np.count_nonzero(map)
        for i in range(1,map.shape[0]):
            if(map[i]==0):
                map[i]=(1/probs)
        mapGrid.data = map*100
        navi.publish(mapGrid)
    FirstTime=False
    my_lock.release()

#Sensor Callback
def callback_sensor(data):
    global sensor
    global moveProbs
    global initMoveProbs
    global mapNew
    global geom
    global navi
    global mapGrid
    global maxProbLoc
    global visual
    global marker
    global FirstTime

    my_lock.acquire()
    if(FirstTime == True):
        sensor = data.ranges
    else:
        #Build marker for RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "navigation"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

    
        sensor = data.ranges
        mapNew = getPriorMap()
        mapGrid.data = mapNew
        mapGrid.info.map_load_time = rospy.Time.now()
        maxProbLoc = Point()
        maxProbLoc.x=np.argmax(mapNew)%20
        maxProbLoc.y=np.argmax(mapNew) // 20
        maxProbLoc.z=0
        marker.pose.position.x = np.argmax(mapNew)%20 + 0.5
        marker.pose.position.y = np.argmax(mapNew) // 20 + 0.5
        navi.publish(mapGrid)
        geom.publish(maxProbLoc)
        visual.publish(marker)
    my_lock.release()

#Move Callback
def callback_move(data):
    global move
    global moveProbs
    global initMoveProbs
    my_lock.acquire()
    move = data
    #N,S,W,E
    if(move=='N'):
        moveProbs[0]=initMoveProbs[0]#NORTH
        moveProbs[1]=initMoveProbs[3]#South
        moveProbs[2]=initMoveProbs[1]#West
        moveProbs[3]=initMoveProbs[2]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='S'):
        moveProbs[0]=initMoveProbs[3]#North
        moveProbs[1]=initMoveProbs[0]#South
        moveProbs[2]=initMoveProbs[2]#West
        moveProbs[3]=initMoveProbs[1]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='W'):
        moveProbs[0]=initMoveProbs[2]#North
        moveProbs[1]=initMoveProbs[1]#South
        moveProbs[2]=initMoveProbs[0]#West
        moveProbs[3]=initMoveProbs[3]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='E'):
        moveProbs[0]=initMoveProbs[1]#North
        moveProbs[1]=initMoveProbs[2]#South
        moveProbs[2]=initMoveProbs[3]#West
        moveProbs[3]=initMoveProbs[0]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    my_lock.release()


#Get new Map
def getPriorMap():
    global moveProbs
    map_new = np.copy(map)
    map_new[(map_new==100)|(map_new==-1)]=0
    calc = 1/np.sum(map_new)
    map_new[(map_new > 0) & (map_new < 100)] = map_new[(map_new > 0) & (map_new < 100)] * calc
    map_tmp = np.copy(map_new)
    for pos in range(0,map_new.shape[0]):
        y=pos//20
        x=pos%20
        if(map_new[pos] != 100 and map_new[pos] != -1 and map_new[pos] != 0):
            xi=map_new[pos]*moveProbs[4] #Probability of staying
            #Is there a step East?
            if(x<19):
                xi=xi+map_new[pos+1]*moveProbs[2]             
            #Is there a step West?
            if(x>0):
                xi=xi+map_new[pos-1]*moveProbs[3]
            #Is there a step South?
            if(y>0):
                xi=xi+map_new[pos-20]*moveProbs[0]
            #Is there a step North?
            if(y<19):
                xi=xi+map_new[pos+20]*moveProbs[1]
            l=getLikelihood(pos, map_new)
            map_tmp[pos]=l*xi     
        else:
            map_tmp[pos]=map_new[pos]   
    map_new=map_tmp 
    return map_new

#MOVE_IDS = ['N', 'S', 'W', 'E']
#Sensor = 'S,W,N,E'
#Calculates the multi of the sensorData
def getLikelihood(position, myMap):
    y=position //20
    x=position%20
    #North
    freeSpaceN = True
    counterN = 1
    while(freeSpaceN and y+counterN < 20):
        check = map[position+20*(counterN)]
        if(check == 100 or check == -1):
            freeSpaceN=False
        else:
            counterN+=1
    #South
    freeSpaceS = True
    counterS = 1
    while(freeSpaceS and y-counterS >=0):
        check = map[position-20*(counterS)]
        if(check == 100 or check == -1):
            freeSpaceS=False
        else:
            counterS+=1

    #West
    freeSpaceW = True
    counterW = 1
    while(freeSpaceW and position-(counterW) >= 0 and (x-counterW)>=0):
        check = map[position-(counterW)]
        if(check == 100 or check == -1):
            freeSpaceW=False
        else:
            counterW+=1
    #East
    freeSpaceE = True
    counterE = 1
    while(freeSpaceE and position+counterE < map.shape[0] and x+counterE<20):
        check = map[position+counterE]
        if(check == 100 or check == -1):
            freeSpaceE=False
        else:
            counterE+=1
    #Calculate Diff
    res=np.array([counterS,counterW,counterN,counterE])
    diff = np.abs(sensor-res)
    mult = 1
    if max(diff)>1:
        return 0
    for val in diff:
        if val == 0:
            mult*=0.8
        if val == 1:   
            mult*=0.1
    return  mult


def localize():
    global moveProbs
    global initMoveProbs
    global my_lock
    global FirstTime
    global geom
    global navi
    global mapGrid
    global maxProbLoc
    global marker
    global visual


    FirstTime=True
    my_lock = Lock()
    mapData = rospy.Subscriber('/map', OccupancyGrid, callback_map)
    sensorData = rospy.Subscriber('/scan', LaserScan, callback_sensor)
    moveData = rospy.Subscriber('/move', String, callback_move)

    geom = rospy.Publisher('/robot_pos', Point , queue_size=10)
    visual = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
    navi = rospy.Publisher('/map_updates', OccupancyGrid, queue_size=10)

    initMoveProbs = rospy.get_param('~robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])
    moveProbs=np.copy(initMoveProbs)
    rospy.init_node('localize', anonymous=True)
    rate = rospy.Rate(2)

    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        subrate.sleep()
        #rospy.signal_shutdown("Done")

if __name__ == '__main__':
    localize()