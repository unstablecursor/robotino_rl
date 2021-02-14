#This class stores variables that are the same across all he different training approaches
class global_parameters:
    
    # Training mode (accepted values: "simple", "goal_distance", "checkpoints")
    # This variable decides which controller node is used during training
    TRAINING_MODE = "simple"

    #maximum amount of episodes in a training run
    NUM_EPISODES = 501

    #network checkpoint save settings
    SAVE_EVERY_NTH_EPISODE = 10
    policy_checkpoint_path = "/home/tobi/checkpoints/cleanup_test/policy_net_10.pth"# "/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models/policy_net_10.pth"
    target_checkpoint_path = "/home/tobi/checkpoints/cleanup_test/target_net_10.pth"# "/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models/target_net_10.pth"
    checkpoint_root = "/home/tobi/checkpoints/cleanup_test"#"/home/deniz/praktikum_final/catkin_ws/src/robotino_rl/models"

    #path for pretrained model used for test run
    trained_model_path = "/home/tobi/checkpoints/cleanup_test/policy_net_10.pth"