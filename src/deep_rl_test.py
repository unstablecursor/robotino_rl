#!/usr/bin/env python3

#this script is used for running a previously trained policy network

import os
import numpy as np

import torch

import rospy

from sensor_msgs.msg import LaserScan

from std_srvs.srv import Empty

from parameters.global_parameters import global_parameters
from components.network import DQN

from controllers.controller_with_goal import ControllerNode
from controllers.controller_simple import SimpleControllerNode
from controllers.controller_with_checkpoints import ControllerNodeCheckpoint

from parameters.parameters_simple import parameters_simple
from parameters.parameters_goal_distance import parameters_goal_distance
from parameters.parameters_checkpoints import parameters_checkpoints

#load the correct parameters
if global_parameters.TRAINING_MODE == "simple":
    param = parameters_simple()

elif global_parameters.TRAINING_MODE == "goal_distance":
    param = parameters_goal_distance()

elif global_parameters.TRAINING_MODE == "checkpoints":
    param = parameters_checkpoints()


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
reset_simulation()

#load the needed controller_node
rospy.init_node('controller_node')

controller_node = None

if global_parameters.TRAINING_MODE == "simple":  
    controller_node = SimpleControllerNode()

elif global_parameters.TRAINING_MODE == "goal_distance":  
    controller_node = ControllerNode()

elif global_parameters.TRAINING_MODE == "checkpoints":
    controller_node = ControllerNodeCheckpoint()


#load a trained policy net
policy_net = DQN(param.N_INPUTS, param.FIRST_LAYER, param.SECOND_LAYER, param.THIRD_LAYER, param.N_ACTIONS).to(device)

print("loading policy network")
policy_checkpoint = torch.load(global_parameters.trained_model_path)
policy_net.load_state_dict(policy_checkpoint['model_state_dict'])

#run the loaded network by always using the advised action for each state
while True:
    state=controller_node.get_env()
    
    if min(state) < param.COLLISION_DISTANCE:
        print("failed: hit a wall")
        controller_node.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        controller_node.reset_simulation()
    
    t = policy_net(state.to(device))
    action = torch.tensor([[t.max(0)[1]]], device=device, dtype=torch.long)
    controller_node.use_action(action.item())