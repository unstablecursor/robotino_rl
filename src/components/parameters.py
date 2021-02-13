# Robot velocity command params
SPEED_X = 1.0
SPEED_Y = 1.0
ANGULAR_Z = 1.0

# Network parameters
FIRST_LAYER = 100
SECOND_LAYER = 200
THIRD_LAYER = 100

# Hyper parameters
REPLAY_MEM_SIZE = 100000
BATCH_SIZE = 64
GAMMA = 0.99
EPS_START = 0.99
EPS_END = 0.05
EPS_DECAY = 5000
TARGET_UPDATE = 10
BETA = 1.0
NUM_EPISODES = 100
LEARNING_RATE = 1e-3


# Input processing values
# Gets value from every 7th value. Make sure it is divisible to 245
INPUT_SLICE_SIZE = 7 
N_INPUTS = 245 // INPUT_SLICE_SIZE 
N_ACTIONS = 3

# Change this values to activate different algorithms
INPUT_PROCESSING = "max"
REWARD_STYLE = "punish_close_walls"
LOSS_TYPE = "huber_loss"

PAUSE_SIM = True

GOAL_ACCEPT_DISTANCE = 1.0
COLLISION_DISTANCE = 0.25

USE_GOAL_DISTANCE_AS_INPUT = False


SAVE_EVERY_NTH_EPISODE = 50
policy_checkpoint_path = None# "/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models/policy_net_10.pth"
target_checkpoint_path = None# "/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models/target_net_10.pth"
checkpoint_root = "/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models"