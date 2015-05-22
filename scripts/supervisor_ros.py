import logging; logger = logging.getLogger("hidden")

from copy import copy

#import sys
import rospy
import json
import sys
from std_msgs.msg import Empty,String
from metal.msg import StnVisu, ActionVisu, RepairMsg, MaSTNUpdate, StnArc
from metal.srv import AleaAction
from supervisor import Supervisor,State

#from mastn_execution.srv import StnVisu

class SupervisorRos(Supervisor):
    def __init__ (self, inQueue, outQueue, planStr, startEvent, stopEvent, agent = None, pddlFiles=None, useMaSTN=False):
        if agent is None:
            logger.error("Cannot repair with ROS without an agent name")
            sys.exit(1)
        self.repair_sub = rospy.Subscriber("hidden/repair/in", RepairMsg, self.repairCallback)
        self.repair_pub = rospy.Publisher("hidden/repair/out", RepairMsg, queue_size=10)
        
        self.stnvisu_pub = rospy.Publisher('/hidden/stnvisu', StnVisu, queue_size=10)
        
        self.mastn_pub = rospy.Publisher('hidden/mastnUpdate/out', MaSTNUpdate, queue_size=10) 
        self.mastn_sub = rospy.Subscriber('hidden/mastnUpdate/in', MaSTNUpdate, self.mastnUpdate) 
        
        self.alea_srv = rospy.Service("/%s/alea" % agent, AleaAction, self.aleaReceived)
        
        Supervisor.__init__(self, inQueue, outQueue, planStr, startEvent, stopEvent, agent, pddlFiles, useMaSTN)
        
        self.repairRos = True
        self.useMaSTN = useMaSTN

    def aleaReceived(self,msg):
        #logger.warning(msg)
        try:
            data = json.loads(msg.data)
            
            if type(data) != dict:
                logger.error("Received an ill formated AleaAction service call. Data should be a dict : %s" % data)
                return False
        except ValueError:
            logger.error("Received an AleaAction service call but the filed data is not json-encoded : %s " % msg.data)
            return False
        
        logger.info("Injecting an alea from ROS into the queue")
        data["type"] = "alea"
        data["aleaType"] = msg.aleaType
        self.inQueue.put(data)
        return True

    def init(self, plan, agent):
        Supervisor.init(self, plan, agent)

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
    
        if self.state == State.DEAD:
            return
        
        if type == "repairRequest":
            logger.info("Received a repair request. Pausing the execution")
            self.state = State.REPAIRINGPASSIVE
            self.stnUpdated()
            
            msg = RepairMsg(self.agent, "repairResponse", self.getCurrentTime(), json.dumps(self.plan.getLocalJsonPlan(self.agent)))
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
            self.state = State.RUNNING
            self.stnUpdated()
        else:
            logger.warning("Received unsupported message of type %s from %s : %s" % (type, sender, msg))

    def stnUpdated(self, onlyPast = False):
        data = []
        j = self.plan.getJsonDescription()
        for k,a in self.plan.actions.items():
            if "dummy" in a["name"]:
                continue
            if self.isResponsibleForAction(a):
                
                executed = (self.tp[a["tEnd"]][1] == "past")
                executing = (self.tp[a["tStart"]][1] == "past")
                
                if onlyPast and not executed and not executing:
                    continue
                
                if k in j["actions"] and "children" in j["actions"][k] and len(j["actions"][k]["children"])>0:
                    hierarchical = True
                else:
                    hierarchical = False
                
                timeStart = self.plan.stn.getBounds(a["tStart"])
                timeEnd = self.plan.stn.getBounds(a["tEnd"])

                m = ActionVisu(a["name"], timeStart.lb, timeStart.ub, timeEnd.lb, timeEnd.ub, executed, executing, hierarchical)
                data.append(m)
        
        currentTime = 0
        if self.beginDate >= 0:
            currentTime = self.getCurrentTime()
        self.stnvisu_pub.publish(self.agent, currentTime, State.reverse_mapping[self.state], data)

    def setTimePoint(self, tp, value):
        l = Supervisor.setTimePoint(self, tp, value)
        if self.useMaSTN:
            rospy.logdebug(l)
            u = MaSTNUpdate()
            u.header.stamp = rospy.Time.now()
            for a in l:
                u.arcs.append(StnArc(a[0], a[1], a[2], a[3]))
            self.mastn_pub.publish(u)
            
    def mastnUpdate(self, data):
        if self.useMaSTN:
            for a in data.arcs:
                rospy.logdebug(a)