hack = ['hack']
giskard = ['giskard']
world = ['world']
ros_visualizer = ['ros_visualizer']
collision_scene = ['collision_scene']
robot_group_name = ['robot_name']
fk_pose = world + ['compute_fk_pose']
fk_np = world + ['compute_fk_np']
joint_states = world + ['state']
controlled_joints = ['controlled_joints']

fill_trajectory_velocity_values = ['fill_trajectory_velocity_values']

# goal_params = ['goal_params']
trajectory = ['traj']
debug_trajectory = ['lbA_traj']
time = ['time']
qp_solver_solution = ['qp_solver_solution']
# last_cmd = ['last_cmd']
# collisions = ['collisions']
goal_msg = ['goal_msg']
goals = ['goals']
drive_goals = ['drive_goals']
eq_constraints = ['eq_constraints']
neq_constraints = ['neq_constraints']
derivative_constraints = ['derivative_constraints']
free_variables = ['free_variables']
debug_expressions = ['debug_expressions']

execute = ['execute']
skip_failures = ['skip_failures']
check_reachability = ['check_reachability']
cut_off_shaking = ['cut_off_shaking']
next_move_goal = ['next_move_goal']
number_of_move_cmds = ['number_of_move_cmds']
cmd_id = ['cmd_id']

result_message = ['result_message']
tracking_start_time = ['tracking_start_time']
time_delay = ['time_delay']
dt = ['dt']

# config
tmp_folder = giskard + ['path_to_data_folder']
qp_controller_config = giskard + ['qp_controller_config']
collision_avoidance_config = giskard + ['collision_avoidance_config']
world_config = giskard + ['world_config']
max_derivative = qp_controller_config + ['max_derivative']
goal_package_paths = giskard + ['goal_package_paths']
max_trajectory_length = qp_controller_config + ['max_trajectory_length']
endless_mode = qp_controller_config + ['endless_mode']
action_server_name = giskard + ['action_server_name']

sample_period = qp_controller_config + ['sample_period']
qp_controller = giskard + ['qp_controller']
debug_expressions_evaluated = qp_controller + ['evaluated_debug_expressions']
qp_solver_name = qp_controller_config + ['qp_solver']
prediction_horizon = qp_controller_config + ['prediction_horizon']
retries_with_relaxed_constraints = qp_controller_config + ['retries_with_relaxed_constraints']
retry_added_slack = qp_controller_config + ['added_slack']
retry_weight_factor = qp_controller_config + ['weight_factor']

# behavior tree
tree_manager = ['behavior_tree']
control_mode = tree_manager + ['control_mode']

# collision avoidance
collision_avoidance_configs = collision_scene + ['collision_avoidance_configs']
collision_matrix = ['collision_matrix']
closest_point = ['cpi']
added_collision_checks = ['added_collision_checks']

collision_checker = collision_scene + ['collision_checker_id']

# rnd stuff
timer_collector = ['timer_collector']

# gripper
gripper_controller = ['gripper_controller']
gripper_trajectory = ['gripper_trajectory']

