#! /usr/bin/env python3

"""
Simple ROS node that exit iff :
 - An agent goes into "ERROR" mode
 - All agents are done or dead
 
Useful as a required node in a roslaunch
"""

from threading import RLock

import rospy
from metal.msg import StnVisu

agents = {} #store a boolen, True if the node is done
mutex = RLock()

def getStnVisu(msg):
    global agents
    
    with mutex:
        if msg.agent not in agents:
            agents[msg.agent] = False

        if msg.state.upper() == "ERROR":
            rospy.signal_shutdown("error of %s" % msg.agent)
        elif msg.state.upper() == "DONE" or msg.state.upper() == "DEAD":
            agents[msg.agent] = True
        else:
            agents[msg.agent] = False


        if all(agents.values()):
            rospy.signal_shutdown("all done")


def main():
    rospy.init_node("watcher", anonymous=True)
    subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, getStnVisu)

    rospy.spin()


if __name__=="__main__":
    main()