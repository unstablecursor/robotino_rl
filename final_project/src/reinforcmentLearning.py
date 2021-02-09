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
        self.learnRate = rospy.get_param('~/learnRate', 0.5) #LearnRate
        self.epsilon = rospy.get_param('~/epsilon', 0.8) #Randomness
        self.gamma = rospy.get_param('~/gamma', 0.8) #forQCalc
        self.actionValues = rospy.get_param('~/actionValues', [0.2,0.3,0.4, 0.0]) #Speed0,Speed1,Speed2,SpeedTurn
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 5)
        self.amountBlocks = rospy.get_param('~/amountBlocks', 8)
        self.newVelo = Twist()
        self.velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callbackMove)
        self.lastActionZ = np.array([0])#Z
        self.lastScan = np.zeros((258))
        self.episode =0
    
    def callbackMove(self, Twist):
        self.newVelo = Twist

    def getSpaceState(self,scan):
        stateSpace = np.zeros((1,self.amountBlocks))
        Q=np.zeros((stateSpace.shape[0], self.actions.shape[0]))
        state, stateSpace, Q = self.addNewState(scan, stateSpace, Q)
        return stateSpace

    def transformState(self,state):
        split=state.shape[0]//self.amountBlocks
        newState=np.zeros(self.amountBlocks)
        for i in range(0,self.amountBlocks-1):
            amountHalf = (split*(i+1)-(i*split))//2
            newState[i]  = state[(i*split)+amountHalf]
        amountHalf = (state.shape[0]-((i+1)*split))//2
        newState[self.amountBlocks-1] = state[(i+1)*split+amountHalf]
        return newState

    def addNewState(self,state, stateSpace,Q):
        newState=self.transformState(state)
        stateSpace=np.round(np.append(stateSpace,[newState], axis=0),1) #Grow State Space
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Grow QTable
        return newState, stateSpace, Q

    def getState(self,state, stateSpace, Q):
        x = 0
        if(state.shape[0] != self.amountBlocks):
            state=self.transformState(state)
        state = np.round(state,1)
        for i in stateSpace:
            if((state==i).all()):
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        state, stateSpace, Q = self.addNewState(state,stateSpace, Q)
        return state, stateSpace.shape[0]-1, stateSpace, Q


    def get_distance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 2)
        self.actions = np.array(['Speed0','LeftTurn', 'RightTurn','LeftTurn1', 'RightTurn1']) #See above
        self.stateSpace = self.getSpaceState(self.scan)
        self.QTable = np.zeros((2, self.actions.shape[0] ))
        self.rewards = np.array([0.3,0.1,0.4,0.21, 0.8, 0.2, 20]) # Drive to Wall, DiffGotBetter, Get Away from Wall, Driving, 90DegDiff, DistanceFrontGrew, Lost
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()

    def checkDistance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 2)
        

    
    def makeAction(self, action, actionValues):
        reward = 0
        lost = False
        #print(action)
        if(action == 'Speed0'):
            self.newVelo.angular.z = 0.0
            self.lastActionZ[0] = self.newVelo.angular.z
            self.newVelo.linear.x = actionValues[0]
        elif(action == 'LeftTurn'):
            self.newVelo.angular.z= 0.20
            self.lastActionZ[0] = self.newVelo.angular.z
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'RightTurn'):
            self.newVelo.angular.z = -0.20
            self.lastActionZ[0] = self.newVelo.angular.z
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'LeftTurn1'):
            self.newVelo.angular.z= 0.30
            self.lastActionZ[0] = self.newVelo.angular.z
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'RightTurn1'):
            self.newVelo.angular.z = -0.30
            self.lastActionZ[0] = self.newVelo.angular.z
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'RedoTurn'):
            self.newVelo.angular.z = -2*self.lastActionZ[0] 
            self.lastActionZ[0] = 0
            self.newVelo.linear.x = actionValues[3]
        self.lastScan = np.copy(self.scan)
        self.velo_move.publish(self.newVelo)  #Publish
        self.checkDistance() #checkDistanceForRewards
        left = np.mean(self.scan[12:18])
        right = np.mean(self.scan[226:233])

        leftOld = np.mean(self.lastScan[12:18])
        rightOld = np.mean(self.lastScan[226:233])

        diff = np.abs(left-right)
        oldDiff = np.abs(leftOld-rightOld)
        if(np.amin(self.scan)<=0.18):
            print('Lost Game: Collision')
            reward = reward - self.rewards[6]
            lost=True
        else:
            reward = reward + (0.25-diff)*(self.rewards[4])       
            if(oldDiff>diff):
                reward = reward + self.rewards[1]
            if(self.newVelo.linear.x>0):
                reward = reward + self.rewards[3]
                if(np.amin(self.scan)<=0.35):
                    reward = reward - self.rewards[0]
                    if(np.amin(self.scan)>=np.amin(self.lastScan)):
                        reward = reward + self.rewards[2]
                    else:
                        reward = reward - self.rewards[2]
            else:
                if(self.lastScan[129]< 3 and self.lastScan[129]< 3  and self.scan[129]>=self.lastScan[129]):
                    reward = reward + self.rewards[5]
                elif(self.scan[129]<3  and self.lastScan[129]< 3 and self.scan[129]<self.lastScan[129]):
                    reward = reward - self.rewards[5]
        text='Step:'+str(self.steps)+';'+'diff:'+str(diff)+';'+'0:'+str(self.scan[129])+'Min:'+str(np.amin(self.scan))+';\n'
        with open("Diff.txt", "a") as text_file:
            text_file.write(text)  
            text_file.close()
        return reward, lost

    def learn(self):
        self.steps = 0
        overallReward = 0
        currentState = self.stateSpace[1]
        done = False
        lost = False
        print("Episode", self.episode)
        print("Epsilon", self.epsilon)
        while not done:
            done = False
            
            oldState,oldStateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
            
            if(lost):
                self.episode = self.episode +1
                print("Episode", self.episode)
                print('Steps', self.steps)
                print('overallReward', overallReward)
                if(self.episode %2 == 0):
                    title = 'QTable_'+str(self.episode)+'.out'
                    np.savetxt(title, self.QTable, delimiter=';',fmt='%.8f')   # X is an array
                    title = 'StateSpace_'+str(self.episode)+'.out'
                    np.savetxt(title, self.stateSpace, delimiter=';',fmt='%.2f')   # X is an array
                self.reset_simulation() 
                currentState = self.stateSpace[1]
                self.newVelo.linear.x = 0
                self.newVelo.angular.z = 0
                lost=False
                if(self.episode > 5):
                    self.epsilon = 0.6
                    self.learnRate = 0.5
                if(self.episode > 10):
                    self.epsilon = 0.4
                    self.learnRate = 0.4
                if(self.episode > 20):
                    self.epsilon = 0.2
                    self.learnRate = 0.2
                if(self.episode >= 25):
                    self.epsilon = 0.1
                    self.learnRate = 0.2
                if(self.episode >= 30):
                    print('Test', overallReward)
                    self.epsilon = 0.0
                    self.learnRate = 0.0
                overallReward = 0
                print("Epsilon", self.epsilon)
                self.steps = 0
            if random.uniform(0,1)<self.epsilon:
                actionId = random.randrange(self.actions.shape[0]) 
                chooseAction = self.actions[actionId]      
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
                actionId = np.argmax(self.QTable[stateId, :])
                chooseAction = self.actions[actionId]  
            myReward, lost = self.makeAction(chooseAction,self.actionValues)
            currentState,stateId, self.stateSpace, self.QTable=self.getState(self.scan , self.stateSpace, self.QTable)
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId])
            self.steps = self.steps+1
            overallReward = overallReward + myReward



    def run(self, rate: float = 30):
        print("Start")
        self.get_distance()
        while not rospy.is_shutdown():
            self.learn()

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=30)