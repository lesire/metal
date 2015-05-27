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
from std_msgs.msg import Empty
from roshidden.srv import AleaAction

aleaServices = {}

def getServices():
    global aleaServices
    #get all services
    services = [x[0] for x in rosgraph.masterapi.Master('/mynode').getSystemState()[2]]
    for s in services:
        m = re.match("^/([a-zA-A1-9]+)/executor/alea$", s)
        if m:
            agent = m.groups()[0]
            aleaServices[agent] = rospy.ServiceProxy("/%s/executor/alea" % agent, AleaAction)

#assume each field has type, date, to and data
"""
data = {"0":{"type" : "robotDead", "date":70, "to":"effibot2", "data":{"robot" : "effibot2"}},
        "1":{"type" : "robotDead", "date":72, "to":"ressac1", "data":{"robot" : "effibot2"}},
        "2":{"type" : "delay", "date":10, "to":"effibot1", "data":{"delay" : 10}},
        "3":{"type" : "delay", "date":30, "to":"effibot1", "data":{"delay" : 10}}
        }
"""
data = {}

def launch(m):
    logger.info("Reveived start message")
    
    getServices()
    logger.info("Got agents %d agents: %s" % (len(aleaServices), " ".join(aleaServices.keys())))
    
    for d in data.values():
        if d["to"] not in aleaServices:
            logger.error("Unknown robot : %s. I know %s" % (d["to"], " ".join(aleaServices.keys())))
            continue

        def cb(d):
            logger.info("Calling %s for an alea of type %s with %s" % (d["to"], d["type"], d["data"]))
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
        logger.info("Aleas read from the given file")
    elif rospy.has_param("/hidden/aleas"):
        #get it from ros
        data = json.loads(rospy.get_param("/hidden/aleas"))
        logger.info("Aleas read from the parameter server")
    else:
        logger.error("No aleas given : nothing read from file or from the ros server")

    rospy.Subscriber("/hidden/start", Empty, launch, queue_size = 1 )

    logger.info("Delay module started")
    
    if sys.version_info >= (3,4):
        threading.main_thread().setName("%delay")

    rospy.spin()

if __name__=="__main__":
    main(rospy.myargv()[1:])
