import os
import numpy as np
from threading import Lock

import torch

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from parameters.global_parameters import global_parameters
from parameters.parameters_simple import parameters_simple as param

"""
This class is used in the "simple" training mode.
The robot receives a flat reward for each step it drives incident free and gets punished when it hits a wall/obstacle.
"""
class SimpleControllerNode:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.get_scan)
            
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.scan = LaserScan()
        self.cmd_vel = Twist()
    
    def get_scan(self, msg):
        self.scan = msg
    

    """
    returns the state of the environment that is used to train the NN
    :returns: a subset of self.scan.ranges
    """
    def get_env(self):
        r = self.scan.ranges
        vision_lines = [r[0], r[41], r[82], r[123], r[164], r[205], r[244]]
        return torch.FloatTensor(vision_lines)

    """
    determines the reward function
    :returns: the reward for the current state of the environment
    """
    def get_reward(self):
        #negative reward if the robot collides with a wall/object
        if min(self.scan.ranges) < param.COLLISION_DISTANCE:
            return -1.0
        
        # a flat positive reward for each step where the robot drives collision free
        return 0.3

    """
    performs an action as prescribed by the learning/test-time agent
    :param nr: index of the action to take
    """
    def use_action(self, nr):
        if nr == 0: #forward
            self.cmd_vel.linear.x = param.SPEED_X
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.angular.z = 0.0
        if nr == 1: #right curve
            self.cmd_vel.linear.x = param.SPEED_X
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.angular.z = param.ANGULAR_Z
        if nr == 2: #left curve
            self.cmd_vel.linear.x = param.SPEED_X
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.angular.z = -param.ANGULAR_Z
        self.pub_twist.publish(self.cmd_vel)