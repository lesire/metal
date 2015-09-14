import logging; logger = logging.getLogger("hidden")

from copy import copy

import pystn
import rospy
import json
import sys
from std_msgs.msg import Empty,String
from metal.msg import StnVisu, ActionVisu, RepairMsg, MaSTNUpdate, StnArc
from metal.srv import AleaAction
from supervisor import Supervisor,State
from plan import Plan

class SupervisorRos(Supervisor):
    def __init__ (self, inQueue, outQueue, planStr, startEvent, stopEvent, agent = None, pddlFiles=None):
        if agent is None:
            logger.error("Cannot repair with ROS without an agent name")
            sys.exit(1)
        self.repair_sub = rospy.Subscriber("hidden/repair/in", RepairMsg, self.repairCallback)
        self.repair_pub = rospy.Publisher("hidden/repair/out", RepairMsg, queue_size=10)
        
        self.stnvisu_pub = rospy.Publisher('/hidden/stnvisu', StnVisu, queue_size=10)
        
        self.mastn_pub = rospy.Publisher('hidden/mastnUpdate/out', MaSTNUpdate, queue_size=10) 
        self.mastn_sub = rospy.Subscriber('hidden/mastnUpdate/in', MaSTNUpdate, self.mastnUpdate) 
        
        self.stnupdate_pub = rospy.Publisher('hidden/stnupdate', String, queue_size=10)
        self.stnupdate_pub_latched = rospy.Publisher('hidden/stnupdate', String, queue_size=10, latch=True)

        self.alea_srv = rospy.Service("alea", AleaAction, self.aleaReceived)
        
        Supervisor.__init__(self, inQueue, outQueue, planStr, startEvent, stopEvent, agent, pddlFiles)
       
        self.repairRos = True

        if rospy.has_param("/hidden/ubForCom"):
            f = float(rospy.get_param("/hidden/ubForCom"))
            logger.info("Setting ubForCom to %s from the parameter server" % f)
            self.ubForCom = f
            
        if rospy.has_param("/hidden/ubForTrack"):
            f = float(rospy.get_param("/hidden/ubForTrack"))
            logger.info("Setting ubForTrack to %s from the parameter server" % f)
            self.ubForTrack = f

    def onMissionStart(self):
        if rospy.has_param("/hidden/ubForCom"):
            f = float(rospy.get_param("/hidden/ubForCom"))
            logger.info("Setting ubForCom to %s from the parameter server" % f)
            self.ubForCom = f
            
        if rospy.has_param("/hidden/ubForTrack"):
            f = float(rospy.get_param("/hidden/ubForTrack"))
            logger.info("Setting ubForTrack to %s from the parameter server" % f)
            self.ubForTrack = f

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
        
        if msg.aleaType == "state":
            logger.info("Changing the current state due to a message reveived on the alea topic")
            s = data["state"]
            if not isinstance(s, str):
                logger.error("state field is not a string : " % data)
                return False
            s = s.upper()
            if s not in dir(State):
                logger.error("State %s is unknown" % s)
                return False
            self.state = getattr(State, s)
            return True
        elif msg.aleaType == "sendMastn":
            if not self.plan.stn.isConsistent():
                logger.error("Cannot send a mastn update : the stn is incoherent")
            else:
                self.sendFullMastnUpdate()
                return True

        logger.info("Injecting an alea from ROS into the queue")
        data["type"] = "alea"
        data["aleaType"] = msg.aleaType
        self.inQueue.put(data)
        return True

    def init(self, plan, agent):
        Supervisor.init(self, plan, agent)

    def sendNewStatusMessage(self, type, data = None):
        if data is None:
            data = "{}"
        if not isinstance(data, str):
            logger.error("When sending a new status, the data field is not a string")
        
        if type == "repairRequest":
            logger.info("Sending repair request")
            self.repair_pub.publish(self.agent, "repairRequest", self.getCurrentTime(), data)
        elif type == "repairResponse":
            self.repair_pub.publish(self.agent, "repairResponse", self.getCurrentTime(), data)
        elif type == "repairDone":
            logger.info("Sending a new plan repaired")
            self.repair_pub.publish(self.agent, "repairDone", self.getCurrentTime(), data)
        elif type == "targetFound":
            logger.info("Sending a target found with data %s" % data)
            self.repair_pub.publish(self.agent, "targetFound", self.getCurrentTime(), data)
        else:
            logger.error("Invalid call to sendNewStatusMessage. Type : %s" % type)

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
            if self.state == State.REPAIRINGACTIVE:
                #Another robot is trying to repair. Abort the repair for one of them
                if self.agent < sender:
                    logger.warning("%s is also trying to repair. He has priority. Canceling my repair" % sender)
                    pass #cancel my reparation
                else:
                    logger.warning("%s is also trying to repair. I have priority. Ignoring its message")
                    return
            elif self.state == State.TRACKING:
                p = self.plan.getLocalJsonPlan(self.agent)
                for k in list(p["actions"].keys()):
                    if "executed" not in p["actions"][k] or not "executed" not in p["actions"][k]:
                        Plan.removeAction(p, k)
                p["state"] = "tracking"
                self.sendNewStatusMessage("repairResponse", json.dumps(p))
                return
            elif self.state != State.RUNNING:
                logger.error("Received a repair request not when running. Ignoring it")
                return
            
            logger.info("Received a repair request. Pausing the execution")
            self.state = State.REPAIRINGPASSIVE
            self.sendVisuUpdate()

            self.sendNewStatusMessage("repairResponse", json.dumps(self.plan.getLocalJsonPlan(self.agent)))

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

            if self.state == State.TRACKING:
                return # Ignore the new plan

            self.state = State.RUNNING
            self.sendVisuUpdate()
        elif type == "targetFound":
            self.targetFound(json.loads(msg), selfDetection = False)
        else:
            logger.warning("Received unsupported message of type %s from %s : %s" % (type, sender, msg))

    def sendVisuUpdate(self, onlyPast = False):
        data = []

        currentTime = 0
        if self.beginDate >= 0:
            currentTime = self.getCurrentTime()
        
        with self.mutex:
            if not self.plan.stn.isConsistent():
                if self.state == State.ERROR:
                    self.stnvisu_pub.publish(self.agent, currentTime, State.reverse_mapping[self.state], data)
                return
            
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

        self.stnvisu_pub.publish(self.agent, currentTime, State.reverse_mapping[self.state], data)

    def stnUpdated(self, data):
        if "init" in data:
            self.stnupdate_pub_latched.publish(json.dumps(data, sort_keys=True))
        else:
            self.stnupdate_pub.publish(json.dumps(data, sort_keys=True))
    
    def setTimePoint(self, tp, value):
        Supervisor.setTimePoint(self, tp, value)
        
        l = self.plan.getMastnMsg()
        rospy.logdebug("Updated constraints: %s" % str(l))

        #only send a MaSTN update if the stn is consistent
        if self.plan.stn.isConsistent():
            u = MaSTNUpdate()
            u.header.stamp = rospy.Time.now()
            for a in l:
                u.arcs.append(StnArc(a[0], a[1], a[2], a[3]))

            u.executedNodes = [tp for tp in self.plan.stn.getFrontierNodeIds() if self.tp[tp][1] == "past"]
            
            u.droppedComs = self.droppedComs

            self.mastn_pub.publish(u)

    def sendFullMastnUpdate(self):
        with self.mutex:
            #only send a MaSTN update if the stn is consistent
            if self.plan.stn.isConsistent():
                logger.info("Sending a full MaSTN update")

                u = MaSTNUpdate()
                u.header.stamp = rospy.Time.now()

                for a in self.plan.stn.getMaSTNConstraints():
                    u.arcs.append(StnArc(a[0], a[1], a[2], a[3]))

                u.executedNodes = [tp for tp in self.plan.stn.getFrontierNodeIds() if self.tp[tp][1] == "past"]

                u.droppedComs = self.droppedComs

                self.mastn_pub.publish(u)

    def mastnUpdate(self, data):
        if data._connection_header["callerid"] == rospy.get_name():
            return

        with self.mutex:
            # Check if a com was cancelled
            logger.debug("%s received a message from %s with dropped coms %s" % (self.agent, data._connection_header["callerid"], data.droppedComs))
            for c in data.droppedComs:
                if c not in self.droppedComs:
                    self.dropCommunication(c)
    
            cl = pystn.ConstraintListL()
            dataStnUpdate = []
            for a in data.arcs:
                if a.nodeSource not in self.plan.stn.getNodeIds() and a.nodeSource != "_origin":
                    logger.error("Cannot find %s in stn. Ignoring this constraint" % a.nodeSource)
                    continue
                if a.nodeTarget not in self.plan.stn.getNodeIds() and a.nodeTarget != "_origin":
                    logger.error("Cannot find %s in stn. Ignoring this constraint" % a.nodeTarget)
                    continue

                cl.append(pystn.ConstraintInt(a.nodeSource, a.nodeTarget, a.directValue, a.indirectValue))
                dataStnUpdate.append({"start":a.nodeSource, "end":a.nodeTarget, "lb":a.directValue, "ub":a.indirectValue})

            self.plan.stn.setConstraints(cl)
            self.stnUpdated({"mastnUpdate":dataStnUpdate})

            if not self.plan.stn.isConsistent():
                logger.error("Received an update from %s. When setting the constraints, stn become inconsistent" % (data._connection_header["callerid"]))
                logger.error(data.arcs)
                return

            for n in data.executedNodes:
                if n in self.tp:
                    self.tp[n][1] = "past"
                    self.executedTp[n] = self.plan.stn.getBounds(n).lb
                else:
                    logger.error("Cannot find %s in tps. Ignoring this mastn update" % n)
