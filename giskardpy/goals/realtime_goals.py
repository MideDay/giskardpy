from __future__ import division

from typing import Optional

import rospy
from geometry_msgs.msg import PointStamped

import giskardpy.casadi_wrapper as cas
from data_types.data_types import PrefixName
from giskardpy.goals.pointing import Pointing
from god_map import god_map
from motion_graph.tasks.task import WEIGHT_BELOW_CA


class RealTimePointing(Pointing):

    def __init__(self,
                 tip_link: PrefixName,
                 root_link: PrefixName,
                 topic_name: str,
                 pointing_axis: cas.Vector3 = None,
                 max_velocity: float = 0.3,
                 weight: float = WEIGHT_BELOW_CA,
                 name: Optional[PrefixName] = None,
                 start_condition: cas.Expression = cas.TrueSymbol,
                 hold_condition: cas.Expression = cas.FalseSymbol,
                 end_condition: cas.Expression = cas.FalseSymbol):
        initial_goal = cas.Point3()
        initial_goal.from_xyz(x=1, z=1, reference_frame=PrefixName('base_footprint'))
        super().__init__(tip_link=tip_link,
                         max_velocity=max_velocity,
                         weight=weight,
                         goal_point=initial_goal,
                         root_link=root_link,
                         name=name,
                         pointing_axis=pointing_axis,
                         start_condition=start_condition,
                         hold_condition=hold_condition,
                         end_condition=end_condition)
        self.sub = rospy.Subscriber(topic_name, PointStamped, self.cb)

    def cb(self, data: PointStamped):
        data = god_map.world.transform(self.root, data)
        self.root_P_goal_point = data
