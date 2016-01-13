#!/usr/bin/env python3

import logging; logger = logging.getLogger("hidden")

from hidden import Hidden
from supervisor_ros import SupervisorRos

import threading
import logging
import rospy
from std_msgs.msg import Empty
import os

class HiddenRos(Hidden):
    """
    ROS implementation of the main class to run METAL.
    """

    def __init__(self):
        Hidden.__init__(self)

    def getDefaultPDDL(self):
        """Gets the plan to execute from the ROS parameter server"""
        if rospy.has_param('/hidden/plan'):
            pddlFiles = {"plan":rospy.get_param('/hidden/plan')}
            if rospy.has_param("/hidden/pddl"):
                d = rospy.get_param('/hidden/pddl')
                pddlFiles.update(d)
              
            return pddlFiles
        
        logger.error("Ros Parameter server has no /hidden/plan")
        return None
        
    def waitSignal(self):
        """Wait the the start command sent to the ROS topic /hidden/start"""
        self.sub = rospy.Subscriber("/hidden/start", Empty, lambda x: self.startCallback(x, x))

    def createSupervisor(self, plan, agent, pddlFiles):
        return SupervisorRos(self.q1, self.q2, plan, self.startSupervisor, self.stopSupervisor, agent=agent, pddlFiles=pddlFiles)

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("hidden", log_level=rospy.DEBUG)
    h = HiddenRos()
    rospy.on_shutdown(h.stop)
    h.init(rospy.myargv()[1:])
    h.main()
