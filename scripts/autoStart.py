#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty

import time

def main():
    rospy.init_node("autoStart", anonymous=True, log_level=rospy.DEBUG)

    #TODO listen to stnvisu to wait for all the agents to be alive.
    time.sleep(5)

    pub = rospy.Publisher("/hidden/start", Empty, latch=True, queue_size=10)
    pub.publish()

    time.sleep(5)

    rospy.signal_shutdown("Done")

if __name__=="__main__":
    main()
