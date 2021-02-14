class parameters_goal_distance:

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
    # Start value of epsilon
    EPS_START = 0.99
    # End value of epsilon
    EPS_END = 0.05
    # Decay (in iterations, each action/reward cycle is counted as 1 decay)
    EPS_DECAY = 5000
    # Target network update after X episodes
    TARGET_UPDATE = 10
    # Loss function Beta value (Huber loss default=1.0)
    BETA = 1.0
    # Learning rate for optimizer (default is 1e-2)
    LEARNING_RATE = 1e-3

    # Input processing values
    INPUT_SLICE_SIZE = 7 # Gets value from every 7th value. Make sure it
    N_INPUTS = 245 // INPUT_SLICE_SIZE 
    N_ACTIONS = 3

    # Change this values to activate different algorithms
    INPUT_PROCESSING = "max"
    REWARD_STYLE = "punish_close_walls"
    LOSS_TYPE = "huber_loss"
    
    # Pause simulation during optimization to reduce delay b/w rewards and actions
    PAUSE_SIM = True

    GOAL_ACCEPT_DISTANCE = 1.0
    COLLISION_DISTANCE = 0.25
    # Whether to include goal distance as an input to the network.
    USE_GOAL_DISTANCE_AS_INPUT = False



