#!/usr/bin/env python3

import time

import rospy
from std_msgs.msg import Empty
from roshidden.srv import AleaAction

alea = {}

def launch(m):
    print("Reveived start message")
    time.sleep(65)

    alea["effibot2"]("robotDead", "{\"robot\":\"effibot2\"}")
    time.sleep(5)

    #alea["effibot1"]("robotDead", "{\"robot\":\"effibot2\"}")
    alea["ressac1"]("robotDead", "{\"robot\":\"effibot2\"}")
    

def main():
    rospy.init_node("delay", anonymous=True)
    rospy.Subscriber("/hidden/start", Empty, launch, queue_size = 1 )

    for robot in ["effibot1", "effibot2", "ressac1", "ressac2"]:
        alea[robot] = rospy.ServiceProxy("/%s/alea" % robot, AleaAction)

    rospy.spin()

if __name__=="__main__":
    main()
