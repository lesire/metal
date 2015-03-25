from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")

import rospy
from hidden.srv import *

class ROSActionExecutor(AbstractActionExecutor):
    def __init__(self, agentName):
        self.name = agentName
        self._in_com = False
        rospy.Service('communicate', CommunicateAction, self.communicate)
        logger.info("ROS executor initialized")

    def _in_communication(self, com):
        if self._in_com:
            return com.first_robot == self._com.first_robot \
                and com.second_robot == self._com.second_robot \
                and com.first_position == self._com.first_position \
                and com.second_position == self._com.second_position
        else:
            return False

    def communicate(self, req):
        return CommunicateActionResponse(self._in_communication(req))

    def update(self):
        if self._in_com:
            com = rospy.ServiceProxy(teammate + "/communicate", CommunicateAction)
            try:
                ack = com(self._com)
                if ack.success:
                    self._in_com = False
                    self._com_cb(ack.success)
            except:
                pass

    # move-aav ressac2 pt_aav_16239_-6582 pt_aav_22229_-2588
    def move_aav(self, who, a, b, cb, **kwargs):
        cb("ok")

    def move_agv(self, who, a, b, cb, **kwargs):
        cb("ok")

    # observe-aav ressac2 pt_aav_22229_-2588 pt_obs_22229_-2588
    def observe_agv(self, who, point, observation, cb, **kwargs):
        cb("ok")

    def observe_aav(self, who, point, observation, cb, **kwargs):
        cb("ok")

    # communicate-aav-agv ressac1 mana pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate_aav_agv(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
        self.communicate_aav_aav(first_robot, second_robot, first_point, second_point, cb, **kwargs)

    # communicate-aav-aav ressac1 ressac2 pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate_aav_aav(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
        if self.name == first_robot:
            teammate = second_robot
        else:
            teammate = first_robot

        self._in_com = True
        self._com = CommunicateActionRequest(first_robot, second_robot, first_point, second_point)
        self._com_cb = cb
