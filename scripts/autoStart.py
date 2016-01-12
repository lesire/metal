#!/usr/bin/env python3

"""
Will read from the plan taken from the ROS param server the list of robots and send the mission start signal when everybody is ready
"""

import logging; logger = logging.getLogger("hidden")

import json
import threading

import rospy
import rostopic
from std_msgs.msg import Empty
from metal.msg import StnVisu

import time

class autoStart:
    def __init__(self):
        self.agents = set()
        self.mutex = threading.RLock()
        self.started = False

        rospy.init_node("autoStart", anonymous=True, log_level=rospy.DEBUG)

        self.pub = rospy.Publisher("/hidden/start", Empty, latch=True, queue_size=10)

        if rospy.has_param('/hidden/plan'):
            planJson = rospy.get_param('/hidden/plan')
            plan = json.loads(planJson)
            self.expectedAgents = set([a["agent"] for a in plan["actions"].values() if "agent" in a])
            logger.info("Autostart waiting for %d agents : %s" % (len(self.expectedAgents), self.expectedAgents))

            self.visu_sub = rospy.Subscriber("/hidden/stnvisu", StnVisu, self.receiveVisu)
        else:
            logger.warning("Autostart can't find the plan. Waiting 5 seconds to start the mission")
            threading.Timer(3, lambda : self.pub.publish()).start()
            threading.Timer(5, lambda : rospy.signal_shutdown("Done")).start()

    def receiveVisu(self, msg):
        if self.started: return

        with self.mutex:
            if msg.state == "INIT":
                self.agents.add(msg.agent)

                if len(self.expectedAgents) == len(self.agents):

                    while rostopic.get_topic_class("/vnet/reload")[1] is None:
                        logger.warning("Cannot find vnet ?!?")
                        logger.warning(rostopic.get_topic_class("/vnet/reload"))
                        time.sleep(5)
                    
                    self.pub_vnet = rospy.Publisher("/vnet/reload", Empty, queue_size=1)
                    threading.Timer(1, lambda : self.pub_vnet.publish()).start()
                
                    logger.info("Autostart has found everyone. Starting in 3 seconds.")
                    self.started = True
                    threading.Timer(3, lambda : self.pub.publish()).start()
                    threading.Timer(5, lambda : rospy.signal_shutdown("Done")).start()

def main():
    sh = logging.StreamHandler()
    sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s %(threadName)s %(process)d (%(filename)s:%(lineno)d): %(message)s'))
    logger.addHandler(sh)

    a = autoStart()
    rospy.spin()

if __name__=="__main__":
    main()
