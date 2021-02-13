#!/usr/bin/env python
import os
import math
import random
import numpy as np
from threading import Lock

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
from components.memory import ReplayMemory
from components.network import DQN
from controllers.controller_with_goal import ControllerNode

steps_done = 0

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))

class ReplayMemory(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            t = policy_net(state)
            return torch.tensor([[t.max(0)[1]]])
    else:
        t = torch.tensor([[random.randrange(N_ACTIONS)]], device=device, dtype=torch.long)
        return t


def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
    # detailed explanation). This converts batch-array of Transitions
    # to Transition of batch-arrays.
    batch = Transition(*zip(*transitions))

    # Compute a mask of non-final states and concatenate the batch elements
    # (a final state would've been the one after which simulation ended)
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                          batch.next_state)), device=device, dtype=torch.bool)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
    
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)

    state_batch=torch.reshape(state_batch, (BATCH_SIZE,N_INPUTS))
    # Compute Q(s_t, a) 
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    # Compute V(s_{t+1}) for all next states.
    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    non_final_next_states=torch.reshape(non_final_next_states, (BATCH_SIZE,N_INPUTS))
    next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0].detach()

    # Compute the expected Q values
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch
    # Compute Huber loss
    if LOSS_TYPE == "huber_loss":
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1), beta=BETA)
    else:
        # TODO : Add new loss types here if needed
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1), beta=BETA)

    # Optimize the model
    loss_float = loss.item()
    optimizer.zero_grad()
    loss.backward()
    for param in policy_net.parameters():
        param.data.clamp_(-1, 1)
    optimizer.step()
    return loss_float

# if gpu is to be used
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

policy_net = DQN().to(device)
target_net = DQN().to(device)

if policy_checkpoint_path != None:
    print("loading network")
    #load policy net
    policy_checkpoint = torch.load(policy_checkpoint_path)
    policy_net.load_state_dict(policy_checkpoint['model_state_dict'])
    optimizer.load_state_dict(policy_checkpoint['optimizer_state_dict'])
    
    #load target net
    target_checkpoint = torch.load(target_checkpoint_path)
    target_net.load_state_dict(target_checkpoint['model_state_dict'])
else:
    target_net.load_state_dict(policy_net.state_dict())

target_net.eval()

optimizer = optim.RMSprop(policy_net.parameters(), lr=LEARNING_RATE)
memory = ReplayMemory(REPLAY_MEM_SIZE)

stats_episode_durations = []
stats_losses = []
stats_overall_rewards = []
stats_reward_cumulative = 0

reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

reset_simulation()
if PAUSE_SIM:
    unpause_sim()

rospy.init_node('controller_node')
simple_sim_node = ControllerNode()


for i_episode in range(NUM_EPISODES):
    if i_episode%SAVE_EVERY_NTH_EPISODE == 0 and i_episode != 0:
        print("saving model after episode " + str(i_episode))
        
        #save policy net
        torch.save({
            'model_state_dict': policy_net.state_dict(),
            'optimizer_state_dict': optimizer.state_dict()
            }, checkpoint_root + "/policy_net_" + str(i_episode) + ".pth")
        
        #save target net
        torch.save({
            'model_state_dict': policy_net.state_dict()
            }, checkpoint_root + "/target_net_" + str(i_episode) + ".pth")

    #   Initialize the environment and state
    if i_episode == 0:
        rospy.wait_for_message("scan", LaserScan)
    # Get state
    state = simple_sim_node.get_env()

    for t in count():
        # Select and perform an action
        action = select_action(state)
        # Unpause simulation
        if PAUSE_SIM:
            unpause_sim()
        simple_sim_node.use_action(action.item())
        # Get reward from action
        reward = simple_sim_node.get_reward()
        # Wait for the new scan message to pause the simulation.
        rospy.wait_for_message("scan", LaserScan)
        if PAUSE_SIM:
            pause_sim()
        stats_reward_cumulative += reward
        reward = torch.tensor([reward], device=device)
    
        next_state = simple_sim_node.get_env()

        # Store the transition in memory
        memory.push(state, action, next_state, reward)

        # Move to the next state
        state = next_state
        
        # Perform one step of the optimization (on the target network)
        loss = optimize_model()
        stats_losses.append(loss)

        if reward == -1 and t > 2:
            stats_overall_rewards.append(float(stats_reward_cumulative / (t+1)))
            stats_reward_cumulative = 0
            stats_episode_durations.append(t + 1)
            reset_simulation()
            if PAUSE_SIM:
                pause_sim()
            break
        if reward == +1:
            stats_overall_rewards.append(float(stats_reward_cumulative / (t+1)))
            stats_reward_cumulative = 0
            print(f"reached goal at {i_episode}th episode {t}")
            stats_episode_durations.append(t + 1)
            reset_simulation()
            if PAUSE_SIM:
                pause_sim()
            break
    # Update the target network, copying all weights and biases in DQN
    if i_episode % TARGET_UPDATE == 0:
        target_net.load_state_dict(policy_net.state_dict())

print('Training complete')
