hack = ['hack']
giskard = ['giskard']
world = ['world']
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
dt = ['dt']

# config
general_options = giskard + ['_general_config']
tmp_folder = giskard + ['path_to_data_folder']
execution_config = giskard + ['execution']
max_derivative = execution_config + ['max_derivative']
action_server_name = execution_config + ['action_server_name']
control_mode = execution_config + ['control_mode']
goal_package_paths = execution_config + ['goal_package_paths']
max_trajectory_length = execution_config + ['max_trajectory_length']
endless_mode = execution_config + ['endless_mode']

sample_period = execution_config + ['sample_period']
qp_controller = giskard + ['qp_controller']
debug_expressions_evaluated = qp_controller + ['evaluated_debug_expressions']
qp_solver_name = execution_config + ['qp_solver']
prediction_horizon = execution_config + ['prediction_horizon']
retries_with_relaxed_constraints = execution_config + ['retries_with_relaxed_constraints']
retry_added_slack = execution_config + ['added_slack']
retry_weight_factor = execution_config + ['weight_factor']

# behavior tree
tree_manager = ['behavior_tree']
tree_tick_rate = giskard + ['behavior_tree', 'tree_tick_rate']

# collision avoidance
collision_avoidance_configs = giskard + ['collision_avoidance', '_collision_avoidance_configs']
collision_matrix = ['collision_matrix']
closest_point = ['cpi']
added_collision_checks = ['added_collision_checks']

collision_checker = giskard + ['collision_avoidance', 'collision_checker_id']

# rnd stuff
timer_collector = ['timer_collector']

# gripper
gripper_controller = ['gripper_controller']
gripper_trajectory = ['gripper_trajectory']

