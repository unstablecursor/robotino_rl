# Reinforcement Learning Project - QTable Learning

In the following we want to explain how to run the reinforcement learning code and which parameters can be adjusted. We also want to explain some of the development and problems.

## How to start the learning:
Use the launch-file:  roslaunch finalProject finalProject.launch

finalProject.launch. The world and starting coordinates are choosen in rto-bringup-sim launch file. 

The are several parameters to choose in finalProject.launch

1. load: choose if the robot should learn a new QTable or to use an old one. (QTable.out, stateSpace.out)
2. learnRate: Choose the learnRate of the robot (0=Tets no training)
3. epsilon: Choose the randomnes of the robot 
4. gamma: how much should the oldState influence the newState
5. maxEpisodes: How many episodes should the robot do?
6. epsilonDecrease: The Decrease of epsilon over the episodes

It is also possible to start the script alone: 

rosrun final_project reinforcmentLearning.py

## The Goal
The goal is to let the robot learn to drive safely between two walls on different tracks. The are no obstacles or any goals to reach. It should just avoid driving into a wall.

## The Code in general
In this approach we use QTable Reinforcement Learning (Non-Deep). Basically the agent (our robot), acts(moves) in a fixed environment(track). Every possible position of the robot in the environment is a state. The robot has a fixed set of actions available and for every state there is one action with the highest reward, which gets choosen. It is also important to have random movements, otherwise the robot could never find a better or the best way.

The algorithm in our case: First we check if the next movement will be random. If the movement is random we choose a random action: move forward, turn right or turn left.
We also need our current state, so we check the scanner values and if they are new we add the state to the stateSpace, the collection of all states. We also add a dummy Entry to our QTable which will contain the learned values of the best actions to choose.  Next we will do the choosen action and check in which state we are after the movement. If the robot is too close to a wall we punish him and if he has hit a wall we punish him even more. Punish means give a negative reward. We also give a positive reward if the robot moved forward without turning to prevent zig-zag behaviour and we also reward if the robot "looks" not at a wall (no obstacles on the way). We use the reward to calculate the QTable Entry of the old state for the corresponding action. If we hit a wall we reset the robot and start a new episode. We also decrease the randomnes of the robot, the random movements, with increased episodes. Therefore the robot moves less random the more he learns.
After many training iterations the QTable should be able to guide the robot properly.

## Details
While in theory the training of the robot is pretty simple, everything is very complex as the selection of rewards, actions and parameters is crucial. We tryed to keep it as simple as possible and add more complexity only if neccesary. 

### Actions
We decided to use only three different actions.
Forward: Move forward without any turn to the left or right.

Turn Left: The robot move slightly to the left, it also moves forward very slowly

Turn Right: The robot move slightly to the right, it also moves forward very slowly

It is important that the robot does not stand still and just turn to the right or left. Otherwise it would just stand still because it does not hit a wall which he needs to avoid.


### Reward Functions
The most compelex thing is how to reward the robot.  In terms of values and results. We started simple by reward driving forward, turn left and right. And punish really hard if the robot hits a wall. Unfortunately the robot was not able to finish the track. The reason could be that if the robot hits a wall in state a and state a-1 gets punished for the movement it is may be already to late, meaning that the robot is too close to the wall and even choosing another action would lead to hit the wall. It would be hard for the robot to find a good solution. So we added another reward punishing based on the distance to the wall. This helps but the robot still had problems because the distance to a wall may not change enough after one turn movement. So another reward was added: The distance at the front to the next wall. So if the robot faces in a direction without any obstacle, it gets rewarded. It should encourage the robot to turn more.

Another idea was to add the difference between -90 and 90 degree as a reward, such that the robot learns to drive in the middle of the two walls. However this didn't work well as the robot, if it finds a curve, suddenly has a big difference and sometimes started to turn in the wrong direction to achieve the min distance between the two sides.

This also leads to the assumption that the track heavely influences the behaviour of the robot and generalization might not be so easy or at least training time would increase a lot.


## Problems
While trying to get the robot to learn we encountered many problems we were not aware of in the beginning of the project. We could "fix" many of them by workarounds but some are still there.

### Training Time:
Using every scan point with every possible scane value would lead to such a high number of states that training time would explode. The robot could train for weeks and still have problems visiting the same states more than once. 

We had three different ways to tackle this problem:
1. Round the values (1.1f)
2. Only use 7 datapoints
3. Discretize the datapoints even more (0.3) 

Looking at the QTable we definitly saw that the robot was able to revisite the different states.

Of course all three points need to get tested, to find the best set up.

### Training Time
Training time in general is a big problem as the robot movement is pretty slow, as it needs to be able to react to the scans. 
We also did not know if the algorithm was not working or it just did not train enough. Looking at the QTable can help but as it is still hard to check all possible states by hand.

In addition we needed to test different parameter settings which probably lead to working settings geeting dropped just because they did not learn fast enough.

### Gazebo
Gazebo had big connection problems. Bot problems still exits but setting the max-step rate in the world setting helped at least witrh the lag. The lag is a problem with gazebo reacting to late to the new actions. Often it has hit a wall and the code sends the reset order but gazebo takes it time to react. This behaviour led to misscommunication and to the behaviour that choosing the same action in the same state could lead to different resulting states depending on the reaction time of gazebo.

Another big problem we all had and still have is that gazebo stops working. It only repeats the last action and is not connected to the subscriber/publisher of the script anymore. It usually happens after many iterations but still lead to us no being able to train the robot over night as we need to restart it manually.


All in all we are far away from testing every possible setting as we need to train them for many hours.


## Notes
The code was not testet with the standard scanner as it resulted in -inf scans. We used the method from the Q&A forum. (Disable GPU Scanner something like that)

It is also important to disable the sim-time as nothing worked with in being True.