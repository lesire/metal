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

    # move ressac2 pt_aav_16239_-6582 pt_aav_22229_-2588
    def move(self, who, a, b, cb, **kwargs):
        MORSEActionExecutor.move(self, who, a, b, cb, **kwargs)

    def observe_agv(self, who, point, observation, cb, **kwargs):
        MORSEActionExecutor.observe(self, who, point, observation, cb, **kwargs)

    # communicate ressac1 mana pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
        ROSActionExecutor.communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs)

    def has_communicated(self, first_robot, second_robot, cb, **kwargs):
        ROSActionExecutor.has_communicated(self, first_robot, second_robot, cb, **kwargs)
