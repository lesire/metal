import logging; logger = logging.getLogger("hidden")

from copy import copy

#import sys
import rospy
import json
from std_msgs.msg import Empty,String
from roshidden.msg import StnVisu, ActionVisu
from supervisor import Supervisor

#from mastn_execution.srv import StnVisu

class SupervisorRos(Supervisor):
    def __init__ (self, inQueue, outQueue, planStr, agent = None, pddlFiles=None):
        Supervisor.__init__(self, inQueue, outQueue, planStr, agent, pddlFiles)
        if agent is None:
            logger.error("Cannot repair with ROS without an agent name")
            sys.exit(1)
        self.repair_sub = rospy.Subscriber("/hidden/repair", String, self.repairCallback)
        self.repair_pub = rospy.Publisher('/hidden/repair', String, queue_size=10)
        
        self.stnvisu_pub = rospy.Publisher('/hidden/stnvisu', StnVisu, queue_size=10)

    def init(self, plan, agent):
        Supervisor.init(self, plan, agent)
        #stn = open(str(self.agent) + "_STN.json", "w")
        #stn.write(self.plan.stn.export())
        #self.plan.stn.toMacroSTN()
        #macro_stn = open(str(self.agent) + "_macroSTN.json", "w")
        #macro_stn.write(self.plan.stn.export())

    def sendRepairMessage(self, msg):
        logger.info("Sending repair msg : %s" %  msg)
        self.repair_pub.publish(msg)

    def repairCallback(self, msg):
        logger.info(msg)
        logger.info(type(msg))
        logger.info(dir(msg))
        try:
            data = json.loads(msg.data)
        except TypeError:
            logger.error("Receive a repair message that is not a json string : %s" % msg.data)
            return
        logger.info("Received : %s" % data)
        
        if data["type"] == "repairRequest":
            msg = {"agent":self.agent, "type":"repairResponse", "plan":self.plan.getJsonDescription()}
            logger.info("Received a repair request")

        elif data["type"] == "repairResponse":
            if data["agent"] not in self.repairResponse:
                self.repairResponse[data["agent"]] = data["plan"]
                logger.info("Receive a repair response from %s " % data["agent"])
            else:
                logger.error("Received several response from %s. Keeping only the first one" % data["agent"])
        else:
            logger.warning("Received unsupported message of type %s : %s" % (data["type"], msg))

    def stnUpdated(self):
        data = []
        j = self.plan.getJsonDescription()
        for k,a in self.plan.actions.items():
            if "dummy" in a["name"]:
                continue
            if self.isResponsibleForAction(a):
                executed = (self.tp[a["tEnd"]][1] == "past")
                executing = (self.tp[a["tStart"]][1] == "past")
                
                if k in j["actions"] and "children" in j["actions"][k] and len(j["actions"][k]["children"])>0:
                  hierarchical = True
                else:
                  hierarchical = False
                
                timeStart = self.plan.stn.getBounds(a["tStart"])
                timeEnd = self.plan.stn.getBounds(a["tEnd"])

                m = ActionVisu(a["name"], timeStart.lb, timeStart.ub, timeEnd.lb, timeEnd.ub, executed, executing, hierarchical)
                data.append(m)
        
        self.stnvisu_pub.publish(self.agent, self.getCurrentTime(), data)

