#! /usr/bin/env python3

"""
Simple ROS node that exit iff :
 - An agent goes into "ERROR" mode
 - All agents are done or dead
 
Useful as a required node in a roslaunch
"""

import time
from threading import RLock

import rospy
from metal.msg import StnVisu
from std_msgs.msg import Empty

agents = {} #store a boolean, True if the node is done
mutex = RLock()
timeout = 20*60
endTime = None

def getStnVisu(msg):
    global agents
    
    with mutex:
        if msg.agent not in agents:
            agents[msg.agent] = False

        if msg.state.upper() == "ERROR":
            rospy.signal_shutdown("error of %s" % msg.agent)
        elif msg.state.upper() == "DONE" or msg.state.upper() == "DEAD" or msg.state.upper() == "TRACKING":
            agents[msg.agent] = True
            rospy.loginfo("Watcher : %s is done" % msg.agent)
        else:
            agents[msg.agent] = False


        if all(agents.values()):
            rospy.loginfo("Watcher : everyone is done")
            rospy.signal_shutdown("all done")

def cbStart(msg):
    global endTime
    endTime = time.time() + timeout

def main():
    rospy.init_node("watcher", anonymous=True)
    subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, getStnVisu)
    subscriber_start = rospy.Subscriber("/hidden/start", Empty, cbStart)

    while not rospy.is_shutdown():
        if endTime is not None and time.time() > endTime:
            rospy.logerr("Timeout")
            rospy.signal_shutdown("timeout")
        else:
            time.sleep(1)
    #rospy.spin()


if __name__=="__main__":
    main()