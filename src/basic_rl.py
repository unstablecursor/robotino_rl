#!/usr/bin/env python

import rospy
import rosservice
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
import random
from std_srvs.srv import Empty
import numpy as np

#Actions (To be Change?)
#0: Stay
#1: Speed X
#2: Slow X
#3: Speed Y
#4: Slow Y
#5: Stop

#Rewards:
#0: -100 Hit Wall
#1: +10 Get Faster
#2: -5 Get Slower
#3: -30 Stop
#4: +100 Finish Round

class learnToMove:

    def __init__(self):
        self.learnRate = rospy.get_param('~/learnRate', 0.7) #LearnRate
        self.epsilon = rospy.get_param('~/epsilon', 0.4) #Randomness
        self.gamma = rospy.get_param('~/gamma', 0.9) #forQCalc
        self.actionValues = rospy.get_param('~/actionValues', [0.2,0.3,0.4, 0.2]) #Speed0,Speed1,Speed2,SpeedTurn
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 5)
        self.amountBlocks = rospy.get_param('~/amountBlocks', 11)
        self.newVelo = Twist()
        self.velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callbackMove)
    
    def callbackMove(self, Twist):
        self.newVelo = Twist

    def getSpaceState(self,scan):
        stateSpace = np.zeros((1,self.amountBlocks+1))
        Q=np.zeros((stateSpace.shape[0], self.actions.shape[0]))
        state, stateSpace, Q = self.addNewState(scan, stateSpace, Q)
        return stateSpace

    def transformState(self,state):
        split=state.shape[0]//self.amountBlocks
        newState=np.zeros(self.amountBlocks+1)
        for i in range(0,self.amountBlocks-1):
            block = state[(i*split):(split*(i+1))]
            newState[i]=np.amin(block)
        newState[self.amountBlocks-1] = np.amin(state[(i+1)*split:])
        newState[-1]=self.newVelo.linear.x
        return newState

    def addNewState(self,state, stateSpace,Q):
        newState=self.transformState(state)
        stateSpace=np.append(stateSpace,[newState], axis=0) #Grow State Space
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Grow QTable
        return newState, stateSpace, Q

    def getState(self,state, stateSpace, Q):
        x = 0
        if(state.shape[0] != self.amountBlocks):
            state=self.transformState(state)
        for i in stateSpace:
            if((state==i).all()):
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        state, stateSpace, Q = self.addNewState(state,stateSpace, Q)
        return state, stateSpace.shape[0]-1, stateSpace, Q


    def get_distance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        self.actions = np.array(['LeftTurn', 'Speed0', 'RightTurn', 'Speed1', 'Speed2']) #See above
        self.stateSpace = self.getSpaceState(self.scan)
        self.QTable = np.zeros((2, self.actions.shape[0] ))
        self.rewards = np.array([0.04,0.02,0.01,-0.1, 0.05, 0.1, 0.001]) # outerDistance, InnerDistance, Straight, TooCloseToWall, Speed, ScaleDiff, Reward Steps
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()

    def checkDistance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        #return np.amin(self.scan[61:184])

    
    def makeAction(self, action, actionValues):
        reward = 0
        lost = False
        #print(action)
        if(action =='Speed2'):
            self.newVelo.angular.z = 0.0
            self.newVelo.linear.x = actionValues[2]
        elif(action =='Speed1'):
            self.newVelo.angular.z = 0.0
            self.newVelo.linear.x = actionValues[1]
        elif(action == 'Speed0'):
            self.newVelo.angular.z = 0.0
            self.newVelo.linear.x = actionValues[0]
        elif(action == 'LeftTurn'):
            self.newVelo.angular.z= 0.10
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'RightTurn'):
            self.newVelo.angular.z = -0.10
            self.newVelo.linear.x = actionValues[3]
        self.velo_move.publish(self.newVelo)  #Publish
        self.checkDistance() #checkDistanceForRewards
        if(self.newVelo.linear.x > 0.1):
            reward = reward + self.newVelo.linear.x *self.rewards[4]
        #Check Outer
        if(np.abs(self.scan[33]-self.scan[212])<=0.2):
            reward = reward + self.rewards[0]
        #Punish Difference
        else:
            diff = np.abs(self.scan[33]-self.scan[212])
            reward = reward -diff *self.rewards[5]
        #HitWall
        if(np.amin(self.scan)<=0.15):
            print('Lost Game: Collision')
            lost=True
        if(np.amin(self.scan)<=0.3):
            reward = reward + self.rewards[3]
        reward = reward + self.steps * self.rewards[6]
        return reward, lost

    def learn(self):
        self.steps = 0
        episode = 0
        overallReward = 0
        currentState = self.stateSpace[1]
        done = False
        lost = False
        print("Episode", episode)
        print("Epsilon", self.epsilon)
        while not done:
            done = False
            if(lost):
                episode = episode +1
                print("Episode", episode)
                print('Steps', self.steps)
                print('overallReward', overallReward)
                self.reset_simulation() 
                currentState = self.stateSpace[1]
                self.newVelo.linear.x = 0
                self.newVelo.angular.z = 0
                lost=False
                if(episode > 14):
                    self.epsilon = 0.3
                if(episode > 15):
                    self.epsilon = 0.2
                if(overallReward > 3000 and episode > 20):
                    self.epsilon = 0.01
                overallReward = 0
                print("Epsilon", self.epsilon)
                self.steps = 0
            if random.uniform(0,1)<self.epsilon:
                actionId = random.randrange(self.actions.shape[0]) 
                chooseAction = self.actions[actionId]       
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
                line = self.QTable[stateId, :]
                if(np.count_nonzero(line)==line.shape[0]):
                    actionId = random.randrange(self.actions.shape[0]) 
                else:
                    actionId = np.argmax(self.QTable[stateId, :])
                chooseAction = self.actions[actionId]  
            oldState=np.copy(currentState)
            myReward, lost = self.makeAction(chooseAction,self.actionValues)
            if(lost):
                myReward = -150
            currentState,stateId, self.stateSpace, self.QTable=self.getState(self.scan , self.stateSpace, self.QTable)
            oldState,oldStateId,self.stateSpace, self.QTable=self.getState(oldState, self.stateSpace, self.QTable)
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId])
            self.steps = self.steps+1
            overallReward = overallReward + myReward
            if(self.steps >  15000):
                lost=True



    def run(self, rate: float = 30):
        print("Start")
        self.get_distance()
        while not rospy.is_shutdown():
            self.learn()

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=30)