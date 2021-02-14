#!/usr/bin/env python
import os
import math
import random
import numpy as np

from collections import namedtuple
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

import rospy

from sensor_msgs.msg import LaserScan

from std_srvs.srv import Empty

from parameters.global_parameters import global_parameters
from components.memory import ReplayMemory
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

steps_done = 0

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))

def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = param.EPS_END + (param.EPS_START - param.EPS_END) * math.exp(-1. * steps_done / param.EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            t = policy_net(state)
            return torch.tensor([[t.max(0)[1]]])
    else:
        t = torch.tensor([[random.randrange(param.N_ACTIONS)]], device=device, dtype=torch.long)
        return t


def optimize_model():
    
    #load the correct parameters
    if global_parameters.TRAINING_MODE == "simple":
        param = parameters_simple()

    elif global_parameters.TRAINING_MODE == "goal_distance":
        param = parameters_goal_distance()

    elif global_parameters.TRAINING_MODE == "checkpoints":
        param = parameters_checkpoints()

    if len(memory) < param.BATCH_SIZE:
        return
    transitions = memory.sample(param.BATCH_SIZE)
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

    state_batch=torch.reshape(state_batch, (param.BATCH_SIZE,param.N_INPUTS))
    # Compute Q(s_t, a) 
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    # Compute V(s_{t+1}) for all next states.
    next_state_values = torch.zeros(param.BATCH_SIZE, device=device)
    non_final_next_states=torch.reshape(non_final_next_states, (param.BATCH_SIZE,param.N_INPUTS))
    next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0].detach()

    # Compute the expected Q values
    expected_state_action_values = (next_state_values * param.GAMMA) + reward_batch
    # Compute Huber loss
    if param.LOSS_TYPE == "huber_loss":
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))
    else:
        # TODO : Add new loss types here if needed
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1), beta=param.BETA)

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

policy_net = DQN(param.N_INPUTS, param.FIRST_LAYER, param.SECOND_LAYER, param.THIRD_LAYER, param.N_ACTIONS).to(device)
target_net = DQN(param.N_INPUTS, param.FIRST_LAYER, param.SECOND_LAYER, param.THIRD_LAYER, param.N_ACTIONS).to(device)

optimizer = optim.RMSprop(policy_net.parameters(), lr=param.LEARNING_RATE)

if global_parameters.policy_checkpoint_path != None:
    print("loading network")
    #load policy net
    policy_checkpoint = torch.load(global_parameters.policy_checkpoint_path)
    policy_net.load_state_dict(policy_checkpoint['model_state_dict'])
    optimizer.load_state_dict(policy_checkpoint['optimizer_state_dict'])
    
    #load target net
    target_checkpoint = torch.load(global_parameters.target_checkpoint_path)
    target_net.load_state_dict(target_checkpoint['model_state_dict'])
else:
    target_net.load_state_dict(policy_net.state_dict())

target_net.eval()

memory = ReplayMemory(param.REPLAY_MEM_SIZE)

stats_episode_durations = []
stats_losses = []
stats_overall_rewards = []
stats_reward_cumulative = 0

reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

reset_simulation()
if param.PAUSE_SIM:
    unpause_sim()

rospy.init_node('controller_node')

controller_node = None

if global_parameters.TRAINING_MODE == "simple":  
    controller_node = SimpleControllerNode()

elif global_parameters.TRAINING_MODE == "goal_distance":  
    controller_node = ControllerNode()

elif global_parameters.TRAINING_MODE == "checkpoints":
    controller_node = ControllerNodeCheckpoint()

else:
    print("invalid TRAINING_MODE")

for i_episode in range(global_parameters.NUM_EPISODES):
    if i_episode%global_parameters.SAVE_EVERY_NTH_EPISODE == 0 and i_episode != 0:
        print("saving model after episode " + str(i_episode))
        
        #save policy net
        torch.save({
            'model_state_dict': policy_net.state_dict(),
            'optimizer_state_dict': optimizer.state_dict()
            }, global_parameters.checkpoint_root + "/policy_net_" + str(i_episode) + ".pth")
        
        #save target net
        torch.save({
            'model_state_dict': policy_net.state_dict()
            }, global_parameters.checkpoint_root + "/target_net_" + str(i_episode) + ".pth")

    #   Initialize the environment and state
    if i_episode == 0:
        rospy.wait_for_message("scan", LaserScan)
    # Get state
    state = controller_node.get_env()

    for t in count():
        # Select and perform an action
        action = select_action(state.to(device))
        # Unpause simulation
        if param.PAUSE_SIM:
            unpause_sim()
        controller_node.use_action(action.item())
        # Get reward from action
        reward = controller_node.get_reward()

        #check if the max amount of iterations is reached (just for the "simple" and "checkpoints" TRAINING_MODEs)
        if global_parameters.TRAINING_MODE == "simple" or global_parameters.TRAINING_MODE == "checkpoints":
            if param.MAX_ITERS != None and t >= param.MAX_ITERS:
                print("reached max number of iterations")

                #assign a positive reward if the maximum amount of iterations is reached without collisions
                if param.MAX_ITER_REWARD:
                    reward = +1.0

                reset_simulation()
                if param.PAUSE_SIM:
                    pause_sim()
                break

        # Wait for the new scan message to pause the simulation.
        rospy.wait_for_message("scan", LaserScan)
        if param.PAUSE_SIM:
            pause_sim()
        stats_reward_cumulative += reward
        reward = torch.tensor([reward], device=device)
    
        next_state = controller_node.get_env()

        # Store the transition in memory
        memory.push(state.to(device), action.to(device), next_state.to(device), reward)

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
            if param.PAUSE_SIM:
                pause_sim()
            break
        if global_parameters.TRAINING_MODE == "goal_distance" and reward == +1:
            stats_overall_rewards.append(float(stats_reward_cumulative / (t+1)))
            stats_reward_cumulative = 0
            print(f"reached goal at {i_episode}th episode {t}")
            stats_episode_durations.append(t + 1)
            reset_simulation()
            if param.PAUSE_SIM:
                pause_sim()
            break


    # Update the target network, copying all weights and biases in DQN
    if i_episode % param.TARGET_UPDATE == 0:
        target_net.load_state_dict(policy_net.state_dict())

print('Training complete')
