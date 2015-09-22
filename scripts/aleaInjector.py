#!/usr/bin/env python3

import logging; logger = logging.getLogger("delay")

import argparse
from copy import copy
from functools import partial
import json
import re
from threading import Timer
import threading
import sys


import rospy
import rosgraph
from std_msgs.msg import Empty,String
from metal.srv import AleaAction

aleaServices = {}
aleaSupServices = {}

def getServices():
    global aleaServices
    global aleaSupServices
    #get all services
    services = [x[0] for x in rosgraph.masterapi.Master('/mynode').getSystemState()[2]]
    for s in services:
        m = re.match("^/([a-zA-A1-9]+)/executor/alea$", s)
        if m:
            agent = m.groups()[0]
            aleaServices[agent] = rospy.ServiceProxy("/%s/executor/alea" % agent, AleaAction)
            aleaSupServices[agent] = rospy.ServiceProxy("/%s/alea" % agent, AleaAction)

#assume each field has type, date, to and data
data = {}

def launch(m):
    rospy.loginfo("Reveived start message")
    
    getServices()
    rospy.loginfo("Got agents %d agents: %s" % (len(aleaServices), " ".join(aleaServices.keys())))
    
    if len(data) == 0:
        rospy.loginfo("No alea to inject")
        rospy.signal_shutdown("Nothing to do")
        return
    
    for d in data.values():
        if "type" not in d or "to" not in d or "data" not in d or "date" not in d:
            rospy.logerr("Ill formated alea : %s" % d)
            continue
        
        if d["to"] == "vnet":
            #assume d is like {"to":"vnet", "type":"add", "date":10, data":{"src":"mana", "tgt":"minnie", ...}}

            def cb(d):
                rospy.loginfo("Calling /vnet/%s with %s" % (d["type"], d["data"]))
                pub = rospy.Publisher("/vnet/%s" % d["type"], String, queue_size=10, latch=True)
                pub.publish(json.dumps(d["data"]))
            Timer(d["date"], partial(cb, copy(d))).start()

        elif d["to"] not in aleaServices:
            rospy.logerr("Unknown robot : %s. I know %s" % (d["to"], " ".join(aleaServices.keys())))
            continue
        else:

            if d["type"] in ["sendMastn", "state", "repair"]:
                def cb(d):
                    rospy.loginfo("Calling sup %s for an alea of type %s with %s" % (d["to"], d["type"], d["data"]))
                    aleaSupServices[d["to"]](d["type"], json.dumps(d["data"]))
                Timer(d["date"], partial(cb, copy(d))).start()
            else:
                def cb(d):
                    rospy.loginfo("Calling exec %s for an alea of type %s with %s" % (d["to"], d["type"], d["data"]))
                    aleaServices[d["to"]](d["type"], json.dumps(d["data"]))
                Timer(d["date"], partial(cb, copy(d))).start()
        
    maxTime = max([d["date"] for d in data.values()])
    Timer(maxTime + 1, lambda: rospy.signal_shutdown("Done")).start()
    

def main(argv):
    parser = argparse.ArgumentParser(description='Execute a plan')
    parser.add_argument('--logLevel', type=str, default="info")
    parser.add_argument('--aleaFile', type=open)
    args = parser.parse_args(argv)


    # Configure the logger
    numeric_level = getattr(logging, args.logLevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.logLevel)
    sh = logging.StreamHandler()
    logger.setLevel(numeric_level)
    sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
    logger.addHandler(sh)

    rospy.init_node("delay", anonymous=True)

    # Get alea data
    global data
    if args.aleaFile is not None:
        #read it from file
        data = json.load(args.aleaFile)
        rospy.loginfo("Aleas read from the given file")
    elif rospy.has_param("/hidden/aleas"):
        #get it from ros
        data = json.loads(rospy.get_param("/hidden/aleas"))
        rospy.loginfo("Aleas read from the parameter server")
    else:
        rospy.logerr("No aleas given : nothing read from file or from the ros server")

    rospy.Subscriber("/hidden/start", Empty, launch, queue_size = 1 )

    rospy.loginfo("Alea injector module started")
    
    if sys.version_info >= (3,4):
        threading.main_thread().setName("%delay")

    rospy.spin()

if __name__=="__main__":
    main(rospy.myargv()[1:])
