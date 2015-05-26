#!/usr/bin/env python3

import logging; logger = logging.getLogger("hidden")

from copy import copy
from functools import partial
import json
import re
from threading import Timer


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
data = {"0":{"type" : "robotDead", "date":70, "to":"effibot2", "data":{"robot" : "effibot2"}},
        "1":{"type" : "robotDead", "date":72, "to":"ressac1", "data":{"robot" : "effibot2"}},
        "2":{"type" : "delay", "date":10, "to":"effibot1", "data":{"delay" : 10}},
        "3":{"type" : "delay", "date":30, "to":"effibot1", "data":{"delay" : 10}}
        }

def launch(m):
    print("Reveived start message")
    
    getServices()
    print("Got agents %d agents: %s" % (len(aleaServices), " ".join(aleaServices.keys())))
    
    for d in data.values():
        if d["to"] not in aleaServices:
            print("Unknown robot : %s. I know %s" % (d["to"], " ".join(aleaServices.keys())))
            continue

        def cb(d):
            print("Calling %s for an alea of type %s with %s" % (d["to"], d["type"], d["data"]))
            aleaServices[d["to"]](d["type"], json.dumps(d["data"]))
        Timer(d["date"], partial(cb, copy(d))).start()
        
    maxTime = max([d["date"] for d in data.values()])
    Timer(maxTime + 1, lambda: rospy.signal_shutdown("Done")).start()
    

def main():
    rospy.init_node("delay", anonymous=True)
    rospy.Subscriber("/hidden/start", Empty, launch, queue_size = 1 )

    rospy.spin()

if __name__=="__main__":
    main()
