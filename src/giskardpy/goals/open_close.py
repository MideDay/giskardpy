from __future__ import division

import math
from typing import Optional, Dict

from giskardpy.goals.cartesian_goals import CartesianPose
from giskardpy.goals.goal import Goal, WEIGHT_ABOVE_CA, WEIGHT_BELOW_CA, ForceSensorGoal
from giskardpy.goals.joint_goals import JointPosition
from giskardpy import casadi_wrapper as w


class Open(ForceSensorGoal):
    def __init__(self,
                 tip_link: str,
                 environment_link: str,
                 tip_group: Optional[str] = None,
                 environment_group: Optional[str] = None,
                 goal_joint_state: Optional[float] = None,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Open a container in an environment.
        Only works with the environment was added as urdf.
        Assumes that a handle has already been grasped.
        Can only handle containers with 1 dof, e.g. drawers or doors.
        :param tip_link: end effector that is grasping the handle
        :param environment_link: name of the handle that was grasped
        :param tip_group: if tip_link is not unique, search in this group for matches
        :param environment_group: if environment_link is not unique, search in this group for matches
        :param goal_joint_state: goal state for the container. default is maximum joint state.
        :param weight:
        """
        super().__init__()

        self.weight = weight
        self.tip_link = self.world.search_for_link_name(tip_link, tip_group)
        self.handle_link = self.world.search_for_link_name(environment_link, environment_group)
        self.joint_name = self.world.get_movable_parent_joint(self.handle_link)
        self.joint_group = self.world.get_group_of_joint(self.joint_name)
        self.handle_T_tip = self.world.compute_fk_pose(self.handle_link, self.tip_link)

        _, max_position = self.world.get_joint_position_limits(self.joint_name)
        if goal_joint_state is None:
            goal_joint_state = max_position
        else:
            # goal_joint_state = min(max_position, goal_joint_state)
            goal_joint_state = goal_joint_state

        self.add_constraints_of_goal(CartesianPose(root_link=environment_link,
                                                   root_group=environment_group,
                                                   tip_link=tip_link,
                                                   tip_group=tip_group,
                                                   goal_pose=self.handle_T_tip,
                                                   weight=self.weight))
        self.add_constraints_of_goal(JointPosition(joint_name=self.joint_name.short_name,
                                                   group_name=self.joint_group.name,
                                                   goal=goal_joint_state,
                                                   weight=WEIGHT_BELOW_CA))

    def make_constraints(self):
        pass

    def __str__(self):
        return f'{super().__str__()}/{self.tip_link}/{self.handle_link}'

    def goal_cancel_condition(self):

        #y_force_threshold = 0.0
        #y_force_condition = lambda sensor_values: math.isclose(sensor_values['y_force'], y_force_threshold, abs_tol=0.3)

        #z_force_threshold = 0.0
        #z_force_condition = lambda sensor_values: math.isclose(sensor_values['z_force'], z_force_threshold, abs_tol=0.3)

        #x_torque_threshold = 0.0
        #x_torque_condition = lambda sensor_values: math.isclose(sensor_values['x_torque'], x_torque_threshold, abs_tol=0.3)

        expression = (lambda sensor_values:
                      (math.isclose(sensor_values['y_force'], 0.0, abs_tol=0.3)) and
                      (math.isclose(sensor_values['z_force'], 0.0, abs_tol=0.3)) and
                      (math.isclose(sensor_values['x_torque'], 0.0, abs_tol=0.3)))

        return expression

    def recovery(self) -> Dict:
        recover = {}

        return recover


class Close(Goal):
    def __init__(self,
                 tip_link: str,
                 environment_link: str,
                 tip_group: Optional[str] = None,
                 environment_group: Optional[str] = None,
                 goal_joint_state: Optional[float] = None,
                 weight: float = WEIGHT_ABOVE_CA):
        """
        Same as Open, but will use minimum value as default for goal_joint_state
        """
        super().__init__()
        self.tip_link = tip_link
        self.environment_link = environment_link
        handle_link = self.world.search_for_link_name(environment_link, environment_group)
        joint_name = self.world.get_movable_parent_joint(handle_link)
        min_position, _ = self.world.get_joint_position_limits(joint_name)
        if goal_joint_state is None:
            goal_joint_state = min_position
        else:
            goal_joint_state = max(min_position, goal_joint_state)
        self.add_constraints_of_goal(Open(tip_link=tip_link,
                                          tip_group=tip_group,
                                          environment_link=environment_link,
                                          environment_group=environment_group,
                                          goal_joint_state=goal_joint_state,
                                          weight=weight))

    def make_constraints(self):
        pass

    def __str__(self):
        return f'{super().__str__()}/{self.tip_link}/{self.environment_link}'
