from .ros_executor import ROSActionExecutor
from .morse_executor import MORSEActionExecutor

import logging; logger = logging.getLogger("hidden")

class MORSEROSActionExecutor(MORSEActionExecutor, ROSActionExecutor):
    def __init__(self, agentName):
        MORSEActionExecutor.__init__(self)
        ROSActionExecutor.__init__(self, agentName)

    def update(self):
        MORSEActionExecutor.update(self)
        ROSActionExecutor.update(self)

    # move-aav ressac2 pt_aav_16239_-6582 pt_aav_22229_-2588
    def move_aav(self, who, a, b, cb, **kwargs):
        MORSEActionExecutor.move_aav(self, who, a, b, cb, **kwargs)

    def move_agv(self, who, a, b, cb, **kwargs):
        MORSEActionExecutor.move_agv(self, who, a, b, cb, **kwargs)

    # observe-aav ressac2 pt_aav_22229_-2588 pt_obs_22229_-2588
    def observe_agv(self, who, point, observation, cb, **kwargs):
        MORSEActionExecutor.observe_agv(self, who, point, observation, cb, **kwargs)

    def observe_aav(self, who, point, observation, cb, **kwargs):
        MORSEActionExecutor.observe_aav(self, who, point, observation, cb, **kwargs)

    # communicate-aav-agv ressac1 mana pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate_aav_agv(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
        ROSActionExecutor.communicate_aav_agv(self, first_robot, second_robot, first_point, second_point, cb, **kwargs)

    # communicate-aav-aav ressac1 ressac2 pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate_aav_aav(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
        ROSActionExecutor.communicate_aav_aav(self, first_robot, second_robot, first_point, second_point, cb, **kwargs)

