import logging; logger = logging.getLogger("hidden")

from copy import copy

#import sys
import rospy
import json
import sys
from std_msgs.msg import Empty,String
from roshidden.msg import StnVisu, ActionVisu, RepairMsg
from supervisor import Supervisor

#from mastn_execution.srv import StnVisu

class SupervisorRos(Supervisor):
    def __init__ (self, inQueue, outQueue, planStr, agent = None, pddlFiles=None):
        Supervisor.__init__(self, inQueue, outQueue, planStr, agent, pddlFiles)
        if agent is None:
            logger.error("Cannot repair with ROS without an agent name")
            sys.exit(1)
        self.repair_sub = rospy.Subscriber("/hidden/repair", RepairMsg, self.repairCallback)
        self.repair_pub = rospy.Publisher("/hidden/repair", RepairMsg, queue_size=10)
        
        self.stnvisu_pub = rospy.Publisher('/hidden/stnvisu', StnVisu, queue_size=10)

        self.repairRos = True

    def init(self, plan, agent):
        Supervisor.init(self, plan, agent)
        #stn = open(str(self.agent) + "_STN.json", "w")
        #stn.write(self.plan.stn.export())
        #self.plan.stn.toMacroSTN()
        #macro_stn = open(str(self.agent) + "_macroSTN.json", "w")
        #macro_stn.write(self.plan.stn.export())

    def sendRepairMessage(self, data = None):
        if data is None:
            logger.info("Sending repair request")
            self.repair_pub.publish(self.agent, "repairRequest", self.getCurrentTime(), "{}")

        elif type(data) == str:
            logger.info("Sending a new plan repaired")
            self.repair_pub.publish(self.agent, "repairDone", self.getCurrentTime(), data)
        else:
            logger.error("Invalid call to sendRepairMessage. Type : %s" % type(data))

    def repairCallback(self, data):
        type = data.type
        time = data.time
        sender = data.sender
        msg = data.data

        if self.agent == sender:
            return
        
        if type == "repairRequest":
            logger.info("Received a repair request. Pausing the execution")
            self.inReparation = True

            msg = RepairMsg(self.agent, "repairResponse", self.getCurrentTime(), json.dumps(self.plan.getJsonDescription()))
            self.repair_pub.publish(msg)

        elif type == "repairResponse":
            try:
                plan = json.loads(msg)
            except TypeError:
                logger.error("Receive a repair message with msg not a json string : %s" % msg)
                return
            
            if "repairResponse" not in dir(self):
                return #I'm not currently repairing

            if sender not in self.repairResponse:
                self.repairResponse[sender] = plan
                logger.info("Receive a repair response from %s " % sender)
            else:
                logger.error("Received several response from %s. Keeping only the first one" % sender)
        elif type == "repairDone":
            logger.info("Receiving a new plan to execute from %s" % sender)
            planStr = msg
            self.init(planStr, self.agent)
            self.inReparation = False
        else:
            logger.warning("Received unsupported message of type %s from %s : %s" % (type, sender, msg))

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

