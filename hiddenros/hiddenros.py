#!/usr/bin/env python3

from hidden import Hidden

import threading
import logging
import rospy
from std_msgs.msg import Empty
import os

#logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
#logging.getLogger().addHandler(rospy.impl.rosout.RosOutHandler())
#logging.getLogger().addHandler(rosgraph.roslogging.RosStreamHandler())

class HiddenRos(Hidden):
    def __init__(self):
        Hidden.__init__(self)
        rospy.init_node("hidden", log_level=rospy.INFO)
        rospy.on_shutdown(lambda: os._exit(1))

    def waitSignal(self):
        self.sub = rospy.Subscriber("start", Empty, lambda x: self.startCallback(x, x))

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    h = HiddenRos()
    h.init()
    h.main()

