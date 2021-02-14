import os
import math
import numpy as np
from threading import Lock

import torch

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest

from parameters.global_parameters import global_parameters
from parameters.parameters_checkpoints import parameters_checkpoints as param

"""
This class is used in the "checkpoints" training mode.
The robot gets rewarded for reaching checkpoints set along the track.

This mode needs to be used with the world files that contain a "_cp" in their name.
The red point is the next checkpoint and the black ones denote the position of the other checkpoints.
"""
class ControllerNodeCheckpoint:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.get_scan)
            
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.scan = LaserScan()
        self.cmd_vel = Twist()


        #the service that lets you SET the state of a model within the gazebo environment (including position)
        rospy.wait_for_service ('/gazebo/set_model_state')
        self.set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        #the service that lets you GET the state of a model withing the gazebo environment (including position)
        rospy.wait_for_service ('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        #state requests withg the names of the respective model
        self.robot_model = GetModelStateRequest()
        self.robot_model.model_name='rto-1'

        self.cp_model = GetModelStateRequest()
        self.cp_model.model_name='cp'

        self.cp_index = 0


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
    get the distance between the robot "rto-1" and checkpoint object "cp"
    :returns: distance to the next checkpoint
    """
    def check_cp_distance(self):
        result = self.get_model_srv(self.robot_model)
        robot_pos = result.pose.position

        result = self.get_model_srv(self.cp_model)
        goal_pos = result.pose.position
        
        diff_x = robot_pos.x - goal_pos.x
        diff_y = robot_pos.y - goal_pos.y

        return math.sqrt(diff_x*diff_x + diff_y*diff_y)

    """
    move the checkpoint object to the next checkpoint marker
    """
    def next_checkpoint(self):
        #update the checkpoint index
        self.cp_index = (self.cp_index + 1)%len(param.CHECKPOINT_NAMES)
        
        #get the position of the next checkpoint market in line
        get_request = GetModelStateRequest()
        get_request.model_name = param.CHECKPOINT_NAMES[self.cp_index]
        get_result = self.get_model_srv(get_request)

        #move the checkpoint object to this new location
        set_request = SetModelStateRequest()
        set_request.model_state.model_name = "cp"
        set_request.model_state.pose = get_result.pose

        set_result = self.set_model_srv(set_request)
    
    """
    determines the reward function
    :returns: the reward for the current state of the environment
    """
    def get_reward(self):
        #negative reward if the robot collides with a wall/object
        if min(self.scan.ranges) < param.COLLISION_DISTANCE:
            self.cp_index = 0
            return -1.0
        
        #high positive reward if a checkpoint is reached, then move checkpoint one step further
        elif self.check_cp_distance() < param.CP_ACCEPT_DISTANCE:
            print("reached checkpoint " + str(self.cp_index))
            self.next_checkpoint()
            return +1.0
        
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