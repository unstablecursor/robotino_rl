class parameters_checkpoints:

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

    #this is honestly a small hack, so I don't have to make an automatic search of checkpoint names within a gazebo scene
    #the worlds are created so that this is the order in which the checkpoints are arranged on the track
    CHECKPOINT_NAMES = ['cp_marker', 'cp_marker_0', 'cp_marker_1', 'cp_marker_2', 'cp_marker_3', 'cp_marker_4', 'cp_marker_5', 'cp_marker_6']

    #distance at which a checkpoint is activated
    CP_ACCEPT_DISTANCE = 1.0

    #maximum amount of iterations before reset
    MAX_ITERS = 5000
    MAX_ITER_REWARD = False


