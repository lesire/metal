from .morse_executor import MORSEActionExecutor

import logging; logger = logging.getLogger("hidden")
import os

try:
    from .ros_executor import ROSActionExecutor

    class MORSEROSActionExecutor(MORSEActionExecutor, ROSActionExecutor):
        _name = "morse+ros"

        def __init__(self, agentName, **kwargs):
            MORSEActionExecutor.__init__(self)
            ROSActionExecutor.__init__(self, agentName)

        def _stop(self, action):
            MORSEActionExecutor._stop(self, action)
            ROSActionExecutor._stop(self, action)

        def _get_morse_host(self):
            #Assume morse is running with roscore
            url = os.environ["ROS_MASTER_URI"]
            return url.split("//")[1].split(":")[0]

        def update(self):
            MORSEActionExecutor.update(self)
            ROSActionExecutor.update(self)

        # communicate ressac1 mana pt_aav_10249_-4585 pt_aav_12245_-4585
        def communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
            ROSActionExecutor.communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs)

        def communicate_meta(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
            ROSActionExecutor.communicate_meta(self, first_robot, second_robot, first_point, second_point, cb, **kwargs)

        def has_communicated(self, first_robot, second_robot, cb, **kwargs):
            ROSActionExecutor.has_communicated(self, first_robot, second_robot, cb, **kwargs)
except ImportError:
    logger.warning("Cannot import ROS")
    pass
