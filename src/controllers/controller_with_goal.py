import os
import math
import random
import numpy as np
from threading import Lock


import matplotlib
import matplotlib.pyplot as plt
from collections import namedtuple
from itertools import count
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

import rospy
import rosservice

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

from std_srvs.srv import Empty
from pcimr_simulation.srv import InitPos
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest

from components.parameters import *

class ControllerNode:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()

        self.sub_twist = rospy.Subscriber('/input/cmd_vel', Twist, self.get_and_filter_twist)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.get_scan)
            
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.scan = LaserScan()
        self.robot_twist = Twist()
        self.cmd_vel = Twist()
        
        #the service that lets you get the state of a model withing the gazebo environment (including position)
        rospy.wait_for_service ('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        #state requests withg the names of the respective model
        self.robot_model = GetModelStateRequest()
        self.robot_model.model_name='rto-1'

        self.goal_model = GetModelStateRequest()
        self.goal_model.model_name='goal'


    def get_and_filter_twist(self, msg):
        self.robot_twist = msg
    
    def process_laserscan(self, l):
        if INPUT_PROCESSING == "max":
            x = []
            maxim = 0
            it = 0
            for i in l:
                if i >= maxim:
                    maxim = i
                if it % INPUT_SLICE_SIZE == 0:
                    x.append(maxim)
                    maxim=0
                it+=1
            return x
        else: 
            x = []
            it = 0
            for i in l:
                if it % INPUT_SLICE_SIZE == 0:
                    x.append(i)
                it+=1
            return x
    
    def get_scan(self, msg):
        self.scan = msg
        
    def get_env(self):
        if USE_GOAL_DISTANCE_AS_INPUT:
            x = self.process_laserscan(self.scan.ranges) + [self.check_goal_distance()]
            return torch.FloatTensor(x)
        else:
            x = self.process_laserscan(self.scan.ranges)
            return torch.FloatTensor(x)
    
    #get the distance between the robot "rto-1" and "goal" object
    def check_goal_distance(self):
        result = self.get_model_srv(self.robot_model)
        robot_pos = result.pose.position

        result = self.get_model_srv(self.goal_model)
        goal_pos = result.pose.position
        
        diff_x = robot_pos.x - goal_pos.x
        diff_y = robot_pos.y - goal_pos.y

        return math.sqrt(diff_x*diff_x + diff_y*diff_y)
    
    def get_reward(self):
        if REWARD_STYLE == "punish_close_walls":
            if min(self.scan.ranges) < COLLISION_DISTANCE:
                return -1.0
            elif self.check_goal_distance() < GOAL_ACCEPT_DISTANCE:
                return +1.0
            elif min(self.scan.ranges) > 1.0:
                return -0.005 #- 0.01 * (self.check_goal_distance() - 4.9)
            elif min(self.scan.ranges) > 0.75:
                return -0.05 #- 0.01 * (self.check_goal_distance() - 4.9)
            elif min(self.scan.ranges) > 0.5:
                return -0.1 #- 0.01 * (self.check_goal_distance() - 4.9)       
            else:
                return -0.2 #- 0.01 * (self.check_goal_distance() - 4.9) 
        else:
            if min(self.scan.ranges) < COLLISION_DISTANCE:
                return -1.0
            
            elif self.check_goal_distance() < GOAL_ACCEPT_DISTANCE:
                return +1.0
            
            return 0.3


    def use_action(self, nr):
        if nr == 0:
            self.cmd_vel.linear.x = SPEED_X
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.angular.z = 0.0
        if nr == 1:
            self.cmd_vel.linear.x = SPEED_X / 4
            self.cmd_vel.linear.y = SPEED_Y
            self.cmd_vel.angular.z = 0.0#ANGULAR_Z
        if nr == 2:
            self.cmd_vel.linear.x = SPEED_X / 4
            self.cmd_vel.linear.y = -SPEED_Y
            self.cmd_vel.angular.z = 0.0#-ANGULAR_Z
        self.pub_twist.publish(self.cmd_vel)