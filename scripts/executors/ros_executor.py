from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")

try:
    import rospy
    from metal.srv import *

    class ROSActionExecutor(AbstractActionExecutor):
        _name = "ros"

        def __init__(self, agentName, **kwargs):
            self.name = agentName
            self._in_com = False
            self._out_com = False
            self._has_com = False
            rospy.Service('communicate', CommunicateAction, self._communicate)
            logger.info("ROS executor initialized")

        def _stop(self, action):
            if "communicate" in action["name"]:
                self._in_com = False
                self._out_com = False

        def _in_communication(self, com):
            if self._out_com:
                return com.first_robot == self._com_request.first_robot \
                    and com.second_robot == self._com_request.second_robot \
                    and com.first_position == self._com_request.first_position \
                    and com.second_position == self._com_request.second_position
            else:
                return False

        def _communicate(self, req):
            logger.info("Received communication request " + str(req))
            if self._in_communication(req):
                self._out_com = self._in_com
                return CommunicateActionResponse(True)
            else:
                return CommunicateActionResponse(False)

        def update(self):
            if self._in_com:
                try:
                    ack = self._com_proxy(self._com_request)
                    if ack.success:
                        logger.info("Communication success")
                        self._com_cb(ack.success)
                        self._in_com = False
                        self._has_com = True
                except:
                    pass

        def move(self, who, a, b, cb, **kwargs):
            cb("ok")

        def observe(self, who, point, observation, cb, **kwargs):
            cb("ok")

        def has_communicated(self, first_robot, second_robot, cb, **kwargs):
            cb(self._has_com)

        # communicate ressac1 ressac2 pt_aav_10249_-4585 pt_aav_12245_-4585
        def communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
            if self.name == first_robot:
                teammate = second_robot
            else:
                teammate = first_robot

            self._com_request = CommunicateActionRequest(first_robot, second_robot, first_point, second_point)
            self._com_cb = cb
            self._com_proxy = rospy.ServiceProxy("/" + teammate + "/communicate", CommunicateAction)
            logger.info("Waiting communication  with /" + teammate + "/communicate")
            self._in_com = True
            self._out_com = True
            self._has_com = False
except ImportError:
    logger.warning("Cannot import ROS")
    pass

