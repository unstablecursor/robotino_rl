class parameters_simple:

    # Robot velocity command params
    SPEED_X = 0.5
    SPEED_Y = 0.5
    ANGULAR_Z = 0.5

    # Neural Network parameters
    FIRST_LAYER = 200
    SECOND_LAYER = 200
    THIRD_LAYER = 200

    # Hyper parameters
    REPLAY_MEM_SIZE = 100000
    BATCH_SIZE = 64
    GAMMA = 0.95
    EPS_START = 0.9
    EPS_END = 0.05
    EPS_DECAY = 200
    TARGET_UPDATE = 10
    BETA = 1.0
    LEARNING_RATE = 0.0005

    # Input processing values
    N_INPUTS = 7
    N_ACTIONS = 3

    LOSS_TYPE = "huber_loss"

    PAUSE_SIM = False

    #laser scan distance at which the algorithm regards the robot colliding with a wall/object
    COLLISION_DISTANCE = 0.2

    #maximum amount of iterations before reset
    MAX_ITERS = 5000
    MAX_ITER_REWARD = False



