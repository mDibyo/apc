string strategy                                # Strategy used for planning motion plan
trajectory_msgs/JointTrajectory[] trajectories # Set of trajectories that make up motion plan
geometry_msgs/Point base_target                # Target coordinates of base
std_msgs/Float32 torso_height                  # Target torso height
geometry_msgs/Point head_direction             # Target head direction
string capture_scene                           # Whether to capture an image