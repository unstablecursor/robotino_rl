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

class learnToMove:

    def __init__(self):
        self.learnRate = rospy.get_param('~/learnRate', 0.2) #Learning Rate, rather small because of the amount of steps per episode
        self.epsilon = rospy.get_param('~/epsilon', 0.5) #Amount of random actions in %
        self.epsilonDecrease = rospy.get_param('~/epsilonDecrease', 0.9987) #Amount of random actions in %
        self.gamma = rospy.get_param('~/gamma', 0.9) #How much does the current state influences the former one
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 3000)#How many episodes should get trained
        self.newVelo = Twist() #Current movement (Twist)
        self.velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=2) #Publisher for the movement
        self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callbackMove)#Subscriber to get the movement
        self.episode =0 #intial episode
        self.load = rospy.get_param('~/load', True)#Load QTable or Start new one
    
    #Callback to get current movement
    def callbackMove(self, Twist):
        self.newVelo = Twist

    #Create the initial stateSpace
    def createStateSpace(self,scan):
        stateSpace = np.zeros((1,7)) #Use 7 scanPoints
        Q=np.zeros((stateSpace.shape[0], self.actions.shape[0])) #Initial QTable
        state, stateSpace, Q = self.addNewState(scan, stateSpace, Q)#Add Scan to stateSpace, function returns more than needed
        return stateSpace

    #Tranform scan into state
    def transformState(self,state):
        bins = np.arange(0.0,5.8,0.2)#Build bins for further discretization
        if(state.shape[0] == 7): #Check if the state is valid (only 7 scan points)
            return state
        newState=np.zeros(7)
        newState[0] = state[0]
        newState[1] = state[15]
        newState[2] = state[65]
        newState[3] = state[129]
        newState[4] = state[179]
        newState[5] = state[229]
        newState[6] = state[244]
        binnedState = np.digitize(newState,bins, right=False) #Discretize
        count = 0
        for i in binnedState:
            newState[count] = bins[i-1]
            count = count +1
        return newState

    #Add the new State
    def addNewState(self,state, stateSpace,Q):
        newState=self.transformState(state) #Transform scan into state
        stateSpace=np.round(np.append(stateSpace,[newState], axis=0),1) #Adjust (grow) the stateSpace
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Adjust (grow) the QTable
        return newState, stateSpace, Q

    #Return state-info (id, state, stateSpace, QTable), also add if the state is new
    def getState(self,state, stateSpace, Q):
        x = 0
        if(state.shape[0] != 7): #Check if state is valid
            state=self.transformState(state)
        state = np.round(state,1)
        for i in stateSpace:
            if((state==i).all()): #Find state in stateSpace
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        state, stateSpace, Q = self.addNewState(state,stateSpace, Q) #If the state is new, add it
        return state, stateSpace.shape[0]-1, stateSpace, Q

    #Initial function, build stateSpace and QTable
    def init(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        self.actions = np.array(['Forward','LeftTurn', 'RightTurn']) #Actions to choose from, drive forward or to the left/right
        if(self.load == False): #Check if a QTable and SpaceState is loaded
            self.stateSpace = self.createStateSpace(self.scan)
            self.QTable = np.zeros((self.stateSpace.shape[0], self.actions.shape[0] ))
        else: #Create new stateSpace and or QTable
            self.stateSpace = np.loadtxt('StateSpace_Track1.out', delimiter=';')
            self.QTable = np.loadtxt('QTable_Track1.out', delimiter=';')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty) #Reset Gazebo initially (security reasons)
        self.reset_simulation()
    
    #get the new scan
    def checkDistance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        
    #performs an action and gets rewarded
    def makeAction(self, action):
        reward = 0
        lost = False #Lost = did it hit a wall?
        if(action == 'Forward'): #Drive forward only
            self.newVelo.angular.z = 0.0
            self.newVelo.linear.x = 0.2
            reward = reward + 5
        elif(action == 'LeftTurn'): #Turn slightly to the left
            self.newVelo.angular.z= 0.30
            reward = reward + 1
            self.newVelo.linear.x = 0.05
        elif(action == 'RightTurn'): #Turn slightly to the right
            self.newVelo.angular.z = -0.30
            reward = reward + 1
            self.newVelo.linear.x = 0.05
        self.velo_move.publish(self.newVelo)  #Publish the movement
        self.checkDistance() #get the new scan
        left = self.scan[15] #Left scan
        right = self.scan[229]#Right scan
        #check for inf
        if(left > 5):
            left = 5
        if(right > 5):
            right = 5
        #check if the robot has hit a wall
        if(np.amin(self.scan)<=0.20):
            print('Lost Game: Collision')
            reward = -200
            lost=True
        else:     
            #reward = reward + (0.3 - np.abs(left-right))*50
            if(np.amin(self.scan)<=0.4): #Check how close it already is to a wall
                reward = reward - ((0.4-np.amin(self.scan))*10)
            if(np.abs(self.scan[129]) > 6): #check for inf
                self.scan[129] = 6
            reward = reward + self.scan[129]*6 #No wall is good
        return reward, lost

    #main function
    def learn(self):
        self.steps = 0 #steps (amount of actions) per episode
        currentState = self.stateSpace[1] #get the initial state
        lost = False
        done=False
        print("Episode", self.episode)
        print("Epsilon", self.epsilon)
        while not done:
            oldState,oldStateId, self.stateSpace, Q=self.getState(currentState, self.stateSpace,self.QTable) #get info about the old state
            if(lost): #next episode
                self.episode = self.episode +1
                print("Episode", self.episode)
                print('Steps', self.steps)
                if(self.episode %10 == 0): #Save the QTable and stateSpace every 10 episodes
                    title = 'QTable_Track1'+'.out'
                    np.savetxt(title, self.QTable, delimiter=';',fmt='%.8f')   # X is an array
                    title = 'StateSpace_Track1'+'.out'
                    np.savetxt(title, self.stateSpace, delimiter=';',fmt='%.1f')   # X is an array
                self.reset_simulation() #Reset Gazebo
                currentState = self.stateSpace[1] #Set back to startin state
                self.newVelo.linear.x = 0 #Initial movement
                self.newVelo.angular.z = 0#Initial movement
                lost=False
                self.epsilon = self.epsilon*self.epsilonDecrease #Decreasing epsilon
                print("Epsilon", self.epsilon)
                self.steps = 0
            if random.uniform(0,1)<self.epsilon: # Check if movement is random or from QTable
                actionId = random.randrange(self.actions.shape[0]) #Get action Id
                chooseAction = self.actions[actionId] #get the action      
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable) #get the state info
                if(np.count_nonzero(currentState) == 0): #check if the state was visited at leats once
                    actionId = random.randrange(self.actions.shape[0]) #random movement
                else:
                    actionId = np.argmax(self.QTable[stateId, :]) #chose movement from QTable
                chooseAction = self.actions[actionId] #Choose the action
            myReward, lost = self.makeAction(chooseAction) #do the action, get reward
            currentState,stateId, self.stateSpace, self.QTable=self.getState(self.scan , self.stateSpace, self.QTable) #get new state
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId]) #update old state on results of new one
            self.steps = self.steps+1 #update steps
            if(self.episode >= self.maxEpisodes): #check if we are done
                done = True

    def run(self, rate: float = 30):
        print("Start")
        r = rospy.Rate(rate)
        self.init()
        self.learn()

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=1000)