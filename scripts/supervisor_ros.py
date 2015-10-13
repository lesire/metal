import logging; logger = logging.getLogger("hidden")

from copy import copy

import threading
import time
import pystn
import rospy
import json
import sys
from std_msgs.msg import Empty,String
from metal.msg import StnVisu, ActionVisu, RepairMsg, MaSTNUpdate, StnArc, StnTp
from metal.srv import AleaAction
from supervisor import Supervisor,State
from plan import Plan, PlanImportError

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
        self.lastPlanSyncRequest = time.time()
        self.lastPlanSyncResponse = time.time()

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
        
        def sendRegularMaSTNUpdate():
            while not rospy.is_shutdown() and self.state != State.DEAD and not self.stopEvent.is_set():
                self.sendFullMastnUpdate()
                rospy.sleep(30)
        threading.Thread(target=sendRegularMaSTNUpdate,name=self.agent+"-mastnbeat").start()

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
            logger.info("Changing the current state due to a message reveived on the alea topic to %s" % data["state"])
            s = data["state"]
            if not isinstance(s, str):
                logger.error("state field is not a string : " % data)
                return False
            s = s.upper()
            if s not in dir(State):
                logger.error("State %s is unknown" % s)
                return False
            
            if self.state == State.TRACKINGCONFIRMATION and s == "TRACKING":
                self.targetFound(self.targetData, selfDetection=False, mustTrack=True)
            elif self.state == State.TRACKINGCONFIRMATION and s == "REPAIRINGACTIVE":
                self.triggerRepair = True
            else:
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
        elif type == "planSyncRequest":
            logger.info("Sending planSync request")
            self.repair_pub.publish(self.agent, "planSyncRequest", self.getCurrentTime(), data)
        elif type == "planSyncResponse":
            logger.info("Sending planSync response")
            self.repair_pub.publish(self.agent, "planSyncResponse", self.getCurrentTime(), data)
        else:
            logger.error("Invalid call to sendNewStatusMessage. Type : %s" % type)

    def repairCallback(self, data):
        if self.state == State.DEAD:
            self.repair_sub.unregister()
            return

        type = data.type
        time = data.time
        sender = data.sender
        msg = data.data

        if self.agent == sender:
            return
    
        if self.state == State.DEAD:
            return
        
        with self.mutex:
            if type == "repairRequest":
                if self.state == State.REPAIRINGACTIVE:
                    #Another robot is trying to repair. Abort the repair for one of them
                    if self.agent < sender:
                        logger.warning("%s is also trying to repair. He has priority. Canceling my repair" % sender)
                        pass #cancel my reparation
                    else:
                        logger.warning("%s is also trying to repair. I have priority. Ignoring its message" % sender)
                        return
                elif self.state == State.TRACKING:
                    p = self.plan.getLocalJsonPlan(self.agent)
                    for k in list(p["actions"].keys()):
                        if "executed" not in p["actions"][k] or not "executed" not in p["actions"][k]:
                            Plan.removeAction(p, k)
                    p["state"] = "tracking"
                    self.sendNewStatusMessage("repairResponse", json.dumps(p))
                    return
                elif self.state not in [State.RUNNING, State.TRACKINGCONFIRMATION, State.DONE]:
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
                
                if self.state in [State.REPAIRINGPASSIVE, State.TRACKING]:
                    self.init(planStr, self.agent)
                else:
                    logger.warning("I'm not is the right state but I received a new plan. Ignoring it, will sync later if needed")
    
                if self.state == State.REPAIRINGPASSIVE:
                    self.state = State.RUNNING
    
                self.sendVisuUpdate()
            elif type == "targetFound":
                with self.mutex:
                    self.targetFound(json.loads(msg), selfDetection = False)
            elif type == "planSyncRequest":
                self.receivePlanSyncRequest(sender)
            elif type == "planSyncResponse":
                msg = json.loads(msg)
                if "plan" not in msg:
                    logger.error("Received an ill-formated planSyncResponse : %s" % msg)
                otherPlan = msg["plan"]
                
                self.receivePlanSyncResponse(sender, otherPlan)
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
        #rospy.logdebug("Updated constraints: %s" % str(l))

        #only send a MaSTN update if the stn is consistent
        if self.plan.stn.isConsistent():
            
            arcs = []
            for a in l:
                arcs.append(StnArc(a[0], a[1], a[2], a[3]))
                
            self.sendMastnUpdate(arcs)

    def startPlanSync(self):
        if time.time() - self.lastPlanSyncRequest < 5:
            logger.info("Do not send a planSync request. Last update was requested at %d, %d seconds before" % (self.lastPlanSyncRequest, time.time() - self.lastPlanSyncRequest))
            return #Do nothing, an update from less than 5 seconds exists
        
        self.lastPlanSyncRequest = time.time()
        self.sendNewStatusMessage("planSyncRequest")
        
    def receivePlanSyncRequest(self, sender):
        if time.time() - self.lastPlanSyncResponse < 5:
            logger.info("Do not send a planSync response. Last update was requested at %d, %d seconds before" % (self.lastPlanSyncResponse, time.time() - self.lastPlanSyncResponse))
            return #Do nothing, an update from less than 5 seconds exists

        self.lastPlanSyncResponse = time.time()
        self.sendNewStatusMessage("planSyncResponse", json.dumps({"plan":self.plan.getJsonDescription()}))


    def receivePlanSyncResponse(self, sender, otherPlan):
        with self.mutex:
            myID = self.plan.ids[-1]
            otherID = otherPlan["ID"]["value"]
            if myID == otherID:
                return

            logger.info("I'm executing plan %s. %s is executing %s" % (self.plan.ids, sender, otherPlan["ID"]))
            
            if self.newPlanToImport is not None:
                newID = json.loads(self.newPlanToImport)["ID"]["value"]
                if newID != otherID:
                    logger.warning("I'm executing %s. I received a new plan %s and a previous plan %s. Ignoring this one" % (myID,otherID,newID))
                else:
                    logger.info("Ignoring this plan since I have already ask for its use")
                return

            if myID in otherPlan["ID"]["parents"]:
                logger.info("The other has repaired and I was not notified. I need to update my plan")
                
                agents = set([a["agent"] for a in otherPlan["actions"].values() if "agent" in a])
                
                logger.info("List of agents in this plan : %s " % agents)
                plansDict = {}
                
                #Prevent the removal of coms time
                otherPlan["current-time"] = (time.time() - self.beginDate)
                p = Plan(json.dumps(otherPlan), self.agent)
                
                #Check that all my communications are still in the plan
                droppedComs = set() #In my current plan
                foreignDroppedCom = set() #In the remote plan
                
                for k,a in self.plan.actions.items():
                    if a["agent"] == self.agent:
                        if k in p.actions and a["name"] == p.actions[k]["name"]:
                            continue #Ok, my action is still here
                        elif k not in p.actions and "communicate" in a["name"]:
                            #self.dropCommunication(a["name"])
                            droppedComs.add(a["name"])
                        else:
                            logger.error("My action %s (%s) is not in the new plan" % (a["name"],k))
                
                for k,a in p.actions.items():
                    if a["agent"] == self.agent:
                        if k in self.plan.actions and a["name"] == self.plan.actions[k]["name"]:
                            continue #Ok, my action is still here
                        if "communicate-meta" in a["name"]:
                            comName = a["name"]
                            if comName in self.droppedComs or a["name"]:
                                #I already dropped this com.
                                logger.warning("They kept a com that I dropped %s (%s)" % (a["name"],k))
                                foreignDroppedCom.add(k)
                            else:
                                logger.error("They added an com action for me %s (%s)" % (a["name"],k))
                                logger.error("%s not in %s" % (comName, self.droppedComs))
                        else:
                            logger.error("They added an action for me %s (%s)" % (a["name"],k))
                            for k1,a1 in self.plan.actions.items():
                                if a1["name"] == a["name"]: logger.error("I have this action with key %s" % k1)
                
                for k,a in self.plan.actions.items():
                    if "communicate-meta" in a["name"] and k not in p.actions:
                        droppedComs.add(a["name"])
                
                for c in droppedComs:
                    self.dropCommunication(c)
                
                if foreignDroppedCom:
                    logger.info("Imported plan before removing a foreign com: %s" % json.dumps(otherPlan))
    
                    for c in foreignDroppedCom:
                        logger.info("Removing action %s (%s)" % (otherPlan["actions"][c], c))
                        logger.info("Length before %s" % len(otherPlan["actions"]))
                        otherPlan = Plan.removeAction(otherPlan, c) #remove the com meta for actions that I dropped
                        logger.info("Length after %s" % len(otherPlan["actions"]))

                    logger.info("Imported plan after removing a foreign com: %s" % json.dumps(otherPlan))
                    p = Plan(json.dumps(otherPlan), self.agent)
                
                for a in agents:
                    if a != self.agent:
                        plansDict[a] = p.getLocalJsonPlan(a)
                plansDict[self.agent] = self.plan.getLocalJsonPlan(self.agent, currentTime=(time.time() - self.beginDate))
                p = Plan.mergeJsonPlans(plansDict, idAgent = sender)
                p["current-time"] = plansDict[self.agent]["current-time"]
                
                # See if the plan is still temporally valid. It could be a problem if I added a ub for a com
                # while the other robot was late : both constraints are problematic. In this case, drop my
                # current com
                try:
                    _ = Plan(json.dumps(p), self.agent)
                except PlanImportError as e:
                    logger.warning("The fused plan will not be valid. Try to drop my current com")
                    for name,_,_ in self.ongoingActions:
                        if "communicate " in name:
                            #Find the com meta name
                            robot1,robot2 = name.split(" ")[1:3]
                            nameIndex = None
                            for k,a in self.plan.actions.items():
                                if a["name"].startswith("communicate-meta %s %s" % (robot1,robot2)) or\
                                   a["name"].startswith("communicate-meta %s %s" % (robot2,robot1)):
                                    name = a["name"]
                                    nameIndex = k
                                    break
                            
                            if nameIndex is None:
                                logger.error("Could not find the index of the com meta action !")
                                self.state = State.ERROR
                                return
                            
                            logger.warning("Dropping %s (%s)" % (name, nameIndex))
                            
                            self.dropCommunication(name)

                            logger.info("Length before %s" % len(otherPlan["actions"]))
                            otherPlan = Plan.removeAction(otherPlan, nameIndex) #remove the com meta for actions that I dropped
                            logger.info("Length after %s" % len(otherPlan["actions"]))
        
                            p = Plan(json.dumps(otherPlan), self.agent)
                            
                            for a in agents:
                                if a != self.agent:
                                    plansDict[a] = p.getLocalJsonPlan(a)
                            plansDict[self.agent] = self.plan.getLocalJsonPlan(self.agent, currentTime=(time.time() - self.beginDate))
                            p = Plan.mergeJsonPlans(plansDict, idAgent = sender)
                            p["current-time"] = plansDict[self.agent]["current-time"]
                
                self.newPlanToImport = json.dumps(p)
                
                #logger.info("Other plans are : %s" % plansDict)
                #logger.info("New plan to import next is %s" % self.newPlanToImport)
                
                return

            elif otherID in self.plan.ids:
                logger.info("I'm more up to date. Do nothing")
                return
            else:
                logger.info("We are not on the same branch : repair the plan")
                self.triggerRepair = True
    
    def sendMastnUpdate(self, arcs):
        u = MaSTNUpdate()
        u.header.stamp = rospy.Time.now()
    
        for a in arcs:
            u.arcs.append(a)
    
        u.sender = self.agent
        
        for k,a in self.plan.getJsonDescription()["actions"].items():
            if ("agent" not in a or a["agent"] == self.agent) and "executed" in a and a["executed"]:
                u.executedActions.append(k)
    
        #executedNodes = [tp for tp in self.plan.stn.getFrontierNodeIds() if self.tp[tp][1] == "past"]
        # Send all executed nodes, not just the frontier, for the same reason that we send the action : if an action is executed we need this information for computing the global plan
        # TODO : have each robot remember the date of its tps ? What happens when one is dead ?
        # Ignore the fake executed node that corresponds to dead robot plans
        executedNodes = [tp for tp in self.tp.keys() if self.tp[tp][1] == "past" and tp != "0-start-dummy init" and (not tp.startswith("1-end") or tp == "1-end-" + self.agent)]
        for tp in executedNodes:
            v = self.plan.stn.getBounds(tp)
            if v.ub != v.lb:
                logger.error("Timepoint %s is executed but stil has some leeway ? %s" % (tp,v))
                continue
            u.executedNodes.append(StnTp(tp, v.lb))
        
        for r in self.agentsDead:
            u.executedNodes.append(StnTp("1-end-%s"%r, -1))
    
        u.droppedComs = self.droppedComs
        
        u.deadRobots = self.agentsDead
        u.planId = self.plan.ids[-1]
    
        self.mastn_pub.publish(u)
    
    def sendFullMastnUpdate(self):
        with self.mutex:
            #only send a MaSTN update if the stn is consistent
            if self.plan.stn.isConsistent():
                logger.info("Sending a full MaSTN update")

                arcs = []
                for a in self.plan.stn.getMaSTNConstraints():
                    arcs.append(StnArc(a[0], a[1], a[2], a[3]))
                
                self.sendMastnUpdate(arcs)

    def mastnUpdate(self, data):
        if self.state == State.DEAD:
            self.mastn_sub.unregister()
            return

        if data._connection_header["callerid"] == rospy.get_name():
            return

        with self.mutex:
                    
            if data.planId != self.plan.ids[-1]:
                logger.warning("I detect an inconsistency in the plan being executed by %s. (%s != %s). Ignoring this update" % (data._connection_header["callerid"], data.planId, self.plan.ids[-1]))
                self.startPlanSync()
                return #ignore this message as it is not relevant anymore. It could cause inconsistency since it was computed on different plans

            for r in data.deadRobots:
                if r not in self.agentsDead:
                    self.agentsDead.append(r)

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

            # Check if a com was cancelled. Must append after updating the temporal constraint as the new imported plan could be invalid with previous constraints
            #logger.info("%s received a message from %s with dropped coms %s" % (self.agent, data._connection_header["callerid"], data.droppedComs))
            for c in data.droppedComs:
                if c not in self.droppedComs:
                    self.dropCommunication(c)

            if not self.plan.stn.isConsistent():
                logger.error("Received an update from %s. When setting the constraints, stn become inconsistent" % (data._connection_header["callerid"]))
                logger.error(data.arcs)
                return

            for n in data.executedNodes:
                if n.tpName in self.tp:
                    if self.tp[n.tpName][1] == "past":
                        continue # I already know this
                    
                    self.tp[n.tpName][1] = "past"
                    
                    bounds = self.plan.stn.getBounds(n.tpName)
                    if n.tpValue >=0 and (n.tpValue > bounds.ub or n.tpValue < bounds.lb):
                        logger.error("Got a foreign tp (%s) executed at %s. But its bounds are %s. Using my bounds" % (n.tpName, n.tpValue, bounds))
                        self.executedTp[n.tpName] = bounds.lb
                    elif n.tpValue <0 or "1-end" in n.tpName:
                        self.executedTp[n.tpName] = n.tpValue
                    else:
                        self.executedTp[n.tpName] = n.tpValue
                        self.plan.setForeignTp(n.tpName, n.tpValue)
                else:
                    # Not one of my tp. But I should bookeep it in case the sender dies and I need its local plan
                    self.plan.setForeignTp(n.tpName, n.tpValue)
                #else:
                #    logger.error("Cannot find %s in tps. Ignoring this mastn constraint" % (n.tpName))

            for k in data.executedActions:
                self.plan.setForeignActionExecuted(k, agentFrom=data.sender)