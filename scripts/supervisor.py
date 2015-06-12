from __future__ import division


from copy import copy
from math import floor
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import logging; logger = logging.getLogger("hidden")

import plan

#Add enum-like code. Taken from https://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.items())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)

State = enum("INIT", "RUNNING", "REPAIRINGACTIVE", "REPAIRINGPASSIVE", "TRACKING", "DEAD", "DONE", "ERROR")


#re-implement the subprocess.call method with a timeout for compatibilty with Python3.2. Taken from Python3.4 source code
def call(*popenargs, timeout=None, **kwargs):
    with subprocess.Popen(*popenargs, **kwargs) as p:
        try:
            return p.wait(timeout=timeout)
        except:
            p.kill()
            p.wait()
            raise

class ExecutionFailed(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

class Supervisor(threading.Thread):
    def __init__ (self, inQueue, outQueue, planStr, startEvent, stopEvent, agent = None, pddlFiles=None):
        self.inQueue = inQueue
        self.outQueue = outQueue
        self.pddlFiles = pddlFiles
        
        self.beginDate = -1
        self.state = State.INIT
        self.repairRos = False
        
        self.allowShorterAction = True # If true, will shorten actions when they finish early. Else, will wait until its nominal length.
        self.ubForCom = True # If true, set an upper bound for communications

        self.mutex = threading.RLock() #Prevent concurent modifications of the plan

        self.agentsDead = []
        self.droppedComs = []

        self.startEvent = startEvent
        self.stopEvent = stopEvent

        threading.Thread.__init__ (self, name="%s-sup" % agent)
        
        #key = (name, startTp, endTp). Value is the JSOn description of the action
        self.ongoingActions = {} # Set it here (and not in init) so it is not reset when the plan is repaired

        self.init(planStr, agent)
        
    
        
    def init(self, planStr, agent):
        with self.mutex:
            self.agent = agent

            try:
                if agent is None:
                    self.plan = plan.Plan(planStr)
                else:
                    self.plan = plan.Plan(planStr, agent)
            except plan.PlanImportError as e:
                self.state = State.ERROR
                self.stnUpdated()
                logger.error(str(e))
                logger.error("Error when trying to import this plan : %s" % planStr)
            
            self.executedTp = {}
            self.tp = {}
            
            logger.info("Building Macro-STN")
            stn = open(str(self.agent) + "_STN.json", "w")
            stn.write(self.plan.stn.export())
            self.plan.stn.toMacroSTN()
            macro_stn = open(str(self.agent) + "_macroSTN.json", "w")
            macro_stn.write(self.plan.stn.export())
    
            for a in self.plan.actions.values():
                if a["name"] == "dummy init":
                    self.tp[a["tStart"]] = [a["name"], "uncontrollable"]
                elif a["name"] == "dummy end":
                    self.tp[a["tStart"]] = [a["name"], "controllable"]
                elif ("dummy" in a["name"]):
                    pass
                elif a["agent"] != self.agent:
                    if a["tStart"] in self.plan.stn.getNodeIds():
                        self.tp[a["tStart"]] = [a["name"], "uncontrollable"]
                    if a["tEnd"] in self.plan.stn.getNodeIds():
                        self.tp[a["tEnd"]] = [a["name"], "uncontrollable"]
                elif a["abstract"]:
                    self.tp[a["tStart"]] = [a["name"], "controllable"]
                    self.tp[a["tEnd"]] =   [a["name"], "controllable"]
                elif a["controllable"]:
                    self.tp[a["tStart"]] = [a["name"], "controllable"]
                    self.tp[a["tEnd"]] =   [a["name"], "controllable"]
                else:
                    self.tp[a["tStart"]] = [a["name"], "controllable"]
                    self.tp[a["tEnd"]] =   [a["name"], "uncontrollable"]
            
            for tp in self.plan.stn.getNodeIds():
                if tp not in self.tp:
                    
                    if tp.startswith("1-end-"):
                        self.tp[tp] = [tp, "uncontrollable"] #end of the local plan of another agent
                    elif tp == "1-endglobal-" + self.agent:
                        self.tp[tp] = ["dummy end", "controllable"] # my end of the global plan
                    elif tp.startswith("1-endglobal-"):
                        self.tp[tp] = [tp, "uncontrollable"] #end of the global plan of another agent
                    else:
                        logger.error("There is a tp that I do not know about : %s" % tp)
                    
            
            for tpName,value in self.plan.absTimes:
                #Ignore nodes that belongs to another robot
                if tpName not in self.plan.stn.getNodeIds(): continue
    
                action = [a for a in self.plan.actions.values() if (a["tStart"] == tpName or a["tEnd"] == tpName) and "dummy" not in a["name"]][0]
    
                if action["executed"]:
                    #action was executed, this is a past action
                    self.executedTp[tpName] = value
                    self.tp[tpName][1] = "past"
                else:
                    self.tp[tpName][1] = "future"
    
                #TODO if this is an action being executed, send a message to the executor
        
            for tpName,value in self.plan.absTimes:
                #Ignore nodes that belongs to another robot
                if tpName not in self.plan.stn.getNodeIds(): continue
    
                action = [a for a in self.plan.actions.values() if (a["tStart"] == tpName or a["tEnd"] == tpName) and "dummy" not in a["name"]][0]
                    
                if action["executed"] and action["tStart"] == tpName and not self.isResponsibleForAction(action) and self.tp[action["tEnd"]][1] != "past":
                    logger.info("When importing, setting the end time of %s" % action["name"])
                    self.tp[action["tEnd"]][1] = "future"
                    if action["abstract"]:
                        pass
                    elif action["controllable"]:
                        self.setTimePoint(action["tEnd"], self.plan.stn.getBounds(action["tEnd"]).lb)
                    else:
                        #logger.info("Setting the end to %s" % int(value + round(1000 * (action["dMin"]))))
                        self.setTimePoint(action["tEnd"], int(value + round(1000 * (action["dMin"]))))
                        
                if not self.plan.stn.isConsistent():
                    logger.error("Error : Invalid stn when setting the time of an absolute tp of the end of an half-executed action : %s" % action["name"])
                    raise ExecutionFailed("Error : Invalid stn when setting the time of an absolute tp of the end of an half-executed action : %s" % action["name"])
        
        self.visuUpdate()
    
    def visuUpdate(self):
        if self.state == State.DEAD:
            self.stnUpdated(onlyPast=True)
            #stop sending messages
        else:
            self.stnUpdated()
            if not self.stopEvent.is_set():
                threading.Timer(1, self.visuUpdate).start() #re-run it in 1 second

    def setTimePoint(self, tpName, value):
        return self.plan.setTimePoint(tpName, value)
    
    def getExecutableTps(self, now = True):
        return filter(lambda tp: self.isTpExecutable(tp, now), self.tp.keys())
    
    def getCurrentTime(self):
        if self.beginDate < 0:
            logger.error("getCurrentTime called before initialisation")

        return int(round(1000 * (time.time() - self.beginDate)))

    def isTpExecutable(self, tp, now = True):
        if self.tp[tp][1] != "controllable" and self.tp[tp][1] != "future":
            return False

        #Check that the lower bound is before the current time
        if now and self.plan.stn.getBounds(str(tp)).lb > self.getCurrentTime():
            return False

        #check that there is no ingoing edge from a non-executed time point
        preconditions = self.plan.stn.getPredecessors(tp)

        if any([tp not in self.executedTp for tp in preconditions]):
            return False

        return True
    
    #returns True if the action is executed by calling the executor. Else it will assume its execution
    def isResponsibleForAction(self, action):
        if action["name"] in ["dummy init", "dummy end"]:
            return True

        if self.agent is None:
            return True
        elif "agent" in action:
            return action["agent"] == self.agent
        elif self.agent in action["name"]:
            return True
        
        return False
    
    def executeTp(self, tp):

        if tp.startswith("1-endglobal-"):
            currentTime = self.getCurrentTime()
            self.executedTp[tp] = currentTime
            self.tp[tp][1] = "past"
            self.setTimePoint(tp, currentTime)

            logger.info("finished the plan")
            self.state = State.DONE
            self.stnUpdated()
            return

        #Retrieve the corresponding action
        a = [a for a in self.plan.actions.values() if (a["tStart"] == tp or a["tEnd"] == tp)][0]
        logger.debug("Executing tp : %s" % tp)
        
        
        if self.tp[tp][1] == "future":
            #If this is a deadline, must use the exact deadline time
            c = self.plan.stn.getBounds(tp)
            if c.lb != c.ub:
                logger.error("A deadline timepoint still has some flexibility ? %s %s" % (tp, c))
            currentTime = c.lb
        elif a["tEnd"] == tp and self.tp[tp][1] == "controllable":
            #If this is the end of an action with temporal constraints
            c = self.plan.stn.getBounds(tp)
            currentTime = self.getCurrentTime()
            currentTime = max(min(currentTime, c.ub), c.lb)
            #currentTime = self.plan.stn.getBounds(tp).lb
        else:
            currentTime = self.getCurrentTime()

        if a["tStart"] == tp:
            self.executeAction(a, currentTime)
        elif a["tEnd"] == tp:
        
            if (a["name"], a["startTp"], a["endTp"]) not in self.ongoingActions:
                logger.warning("When finishing the execution of %s, could not find it in self.ongoingActions" % a["name"])
                logger.warning(self.ongoingActions)
                logger.warning(a)
            else:
                del self.ongoingActions[(a["name"], a["startTp"], a["endTp"])]
                logger.info("Ongoing actions : %s" % self.ongoingActions)

            if self.tp[tp][1] == "controllable":
                
                #End of a controllable action
                self.setTimePoint(tp, currentTime)
                
                if not self.plan.stn.isConsistent():
                    logger.warning("\tError : invalid STN when finishing execution of %s" % a["name"])
                    raise ExecutionFailed("\tInvalid STN when finishing execution of %s" % a["name"])

                if not a["abstract"] and self.isResponsibleForAction(a):
                    logger.info("Stop of action {a} at time {t}".format(a=a["name"],t=currentTime))

                    msg = {"type":"stopAction", "action":copy(a), "time":currentTime}
                    self.outQueue.put(msg)
                
            self.tp[tp][1] = "past"
            self.executedTp[tp] = currentTime
                
        else:
            logger.error("\tError : a timepoint does not match its action %s" % tp)
            raise ExecutionFailed("\tA timepoint does not match its action %s" % tp)

    def executeAction(self, action, currentTime):
        if self.tp[action["tStart"]][1] != "controllable" and self.tp[action["tStart"]][1] != "future":
            logger.error("Cannot execute %s, the status of its start point is %s." % (action["name"], self.tp[action["tStart"]] ))
            return

        if self.isResponsibleForAction(action):
            logger.info("Starting %s at time %f." % (action["name"], currentTime/1000))
        else:
            logger.debug("Starting %s at time %f. Not my action." % (action["name"], currentTime/1000))

        if self.ubForCom and action["name"].startswith("communicate "):
            # Compute an upper bound for this action

            s = copy(self.plan.stn)
            endNode = "1-end-%s" %  self.agent if self.agent is not None else "1-end"
            c = s.getBounds(endNode)
            s.addConstraint(s.getStartId(), endNode, 0, c.lb + 60000) #Aim to finish the plan within 1 minute of its lower bound
            if not s.isConsistent():
                logger.error("When trying to constrain the end timepoint, the stn became inconsistent")
                logger.error("I did not set a deadline for this action")
            else:
                cCom = s.getBounds(action["tEnd"])
                logger.info("The bounds for the end of this com is : %s" % cCom)
                ub =  math.ceil((cCom.ub + cCom.lb)/2)

                #ub = self.plan.stn.getBounds(action["tEnd"]).lb + 10000 #10 seconds
                logger.info("Executing a com action at %.2f. Set its upper bound to %.2f. Max duration : %.2f" % (currentTime/1000, ub/1000, (ub - currentTime)/1000))

                self.plan.addTemporalConstraint(None, action["tEnd"], 0, ub)

                if not self.plan.stn.isConsistent():
                    logger.error("When constraining the end of %s before %d, stn became inconsistent" % (action["name"], ub))
                    self.state = State.ERROR
                    self.stnUpdated()
                    return

                threading.Timer((ub - currentTime)/1000, lambda : self.inQueue.put({"type":"ubAction", "action":action, "date":ub})).start()


        self.executedTp[action["tStart"]] = currentTime
        self.tp[action["tStart"]][1] = "past"
        
        self.setTimePoint(action["tStart"], currentTime)

        if action["name"] == "dummy end":
            #do nothing, the plan is finished only if the 1-endglobal is executed
            return
            
        if self.isResponsibleForAction(action):
            self.ongoingActions[(action["name"], action["startTp"], action["endTp"])] = action

        if not action["abstract"]:
            if not self.isResponsibleForAction(action):
                #This action should not be executed by this robot. Assume someone else will do it
                self.tp[action["tEnd"]][1] = "future"
                if action["controllable"]:
                    self.setTimePoint(action["tEnd"], self.plan.stn.getBounds(action["tEnd"]).lb)
                else:
                    self.setTimePoint(action["tEnd"], currentTime + int(round(1000 * action["dMin"])))

                logger.debug("Action %s is not for this agent. Assume it will last is minimum duration" % action["name"])
            else:
                #send execution order
                msg = {"type":"startAction", "action":copy(action), "time":currentTime}
                
                logger.debug("Sending message : %s" % msg)
                self.outQueue.put(msg)
        
        if not self.plan.stn.isConsistent():
            logger.error("\tError : invalid STN when launching execution of %s" % action["name"])
            raise ExecutionFailed("Invalid STN when launching execution of %s" % action["name"])

    # targetPos is a dict
    def targetFound(self, targetPos = None, notifyTeam = True):
        self.state = State.TRACKING
        
        x,y = 0,0
        if targetPos is not None:
            x = targetPos.get("x", 0)
            y = targetPos.get("y", 0)
        self.outQueue.put({"type":"startAction", "action":{"name":"track %s %s" % (x,y), "dMin":1}, "time":self.getCurrentTime()})
    
        if notifyTeam:
            self.sendNewStatusMessage("targetFound", json.dumps(targetPos))
        
    def endAction(self, msg):
        action = msg["action"]
        tp = action["tEnd"]
        value = msg["time"]

        report = msg.get("report", None)
        
        if isinstance(report, str):
            report = {"type" : report}
        
        if (action["name"], action["startTp"], action["endTp"]) not in self.ongoingActions:
            logger.warning("When finishing the execution of %s, could not find it in self.ongoingActions" % action["name"])
            logger.warning(self.ongoingActions)
            logger.warning(action)
        else:
            del self.ongoingActions[(action["name"], action["startTp"], action["endTp"])]
            logger.info("Ongoing actions : %s" % self.ongoingActions)
            
        if report is None:
            logger.warning("End of action %s without report" % action["name"])
        elif report["type"] == "ok":
            logger.info("End of action %s ok" % action["name"])
        elif report["type"] == "interrupted":
            logger.info("Action %s interrupted by the supervisor" % action["name"])
        else:
            logger.warning("End of action %s with status %s" % (action["name"], report))
            if "target" in report["type"]:
                targetPos = report.get("position", None)
                self.targetFound(targetPos)
            elif report["type"] in ["ko", "broken"]:
                self.state = State.DEAD
                logger.warning("End of the action %s with status %s. Killing myself" % (action["name"], report["type"]))
                return
            else:
                logger.info("Ignoring it")
        
        if action["controllable"]:
            logger.info("Notified of the end of controllable action %s." % action["name"])
            if self.tp[tp][1] != "past":
                logger.error("Notified of the end of controllable action %s." % action["name"])
                logger.error("But the timepoint is not past : %s" % self.tp[tp][1])
            return
        
        logger.info("End of action %s at %s. Status of the tp : %s" % (action["name"], value, self.tp[tp][1]))
        
        #check the scheduled duration of the action
        startTime = self.plan.stn.getBounds(action["tStart"]).lb
        dReal = value - startTime
        dMin = int(action["dMin"]*plan.timeFactor)
        
        if dReal < dMin:
            if self.allowShorterAction:
                logger.warning("An action finished early : %.2f instead of %.2f. Updating its dMin." % (dReal/1000, dMin/1000))
                l = self.plan.stn.setConstraint(action["tStart"], tp, int(floor(dReal)))
                self.plan.mastnMsg = self.plan.mastnMsg + l
            else:
                logger.warning("An action finished early : %.2f instead of %.2f. Will wait" % (dReal/1000, dMin/1000))
                value = startTime + dMin
        
        lb = self.plan.stn.getBounds(tp).lb
        if self.tp[tp][1] == "uncontrollable" and value < lb:
            logger.warning("Finished %s early : %s. Waiting until %s." % (action["name"], value, lb))
            value = lb
            
        logger.info("End of the action %s at %s" % (action["name"], value/1000))
        
        c = self.plan.stn.getBounds(str(tp))

        self.executedTp[tp] = value
        self.tp[tp][1] = "past"
            
        self.setTimePoint(tp, value)
            
        if not self.plan.stn.isConsistent():
            logger.error("\tError : invalid STN when finishing execution of tp %s" % tp)
            raise ExecutionFailed("Invalid STN when finishing execution of tp %s" % tp)

    def checkActionUB(self, msg):
        if "action" not in msg or "date" not in msg:
            logger.error("Ill-formated ubAction message : %s" % msg)

        action = msg["action"]
        date = msg["date"]
        
        if (action["name"], action["startTp"], action["endTp"]) not in self.ongoingActions:
            return # action already finished, nothing to do

        logger.info("Reached the upper bound for %s" % action["name"])

        if "communicate " in action["name"]:
            self.dropCommunication(action["name"])

        if action["controllable"]:
            self.executeTp(action["tEnd"])
        else:
            fakeMsg = {"type":"endAction", "action":action, "time":date, "report":{"type":"interrupted"}}
            self.endAction(fakeMsg)

    # Remove the synchronising action of a communication.
    def dropCommunication(self, comName):
        # remove the communicate meta from the pla
        planJson = self.plan.getJsonDescription()
        
        # match the action also if I'm not the owner
        if "communicate-meta" in comName:
            comMetaName = comName
        else:
            comMetaName = comName.replace("communicate", "communicate-meta")
        l = [(k,a) for k,a in planJson["actions"].items() if set(a["name"].split(" ")) == set(comMetaName.split(" "))]
        if len(l) != 1:
            logger.error("Could not find a single meta action when dropping a com action")
            logger.error(len(l))
            return

        k,a = l[0]

        if "executed" in a and a["executed"]:
            logger.error("Cannot drop %s. %s is executed" % (comName, a["name"]))
            return
        
        logger.info("Dropping communication : %s" % a["name"])

        if not a["name"] in self.droppedComs:
            self.droppedComs.append(a["name"])

        planJson = plan.Plan.removeAction(planJson, k)
        planJson["current-time"] = self.getCurrentTime()

        self.init(json.dumps(planJson), self.agent)
        
        logger.info("Finished importing the new plan when deleting the com-meta action")
        if not self.plan.stn.isConsistent():
            logger.error("Removing the com-meta action invalidated the stn")
        

    def isExecuted(self):
        with self.mutex:
            result = all([tp[1] == "past" for tp in self.tp.values()])
            return result
        
    def printPlan(self):
        info = [(a["name"], self.tp[a["tStart"]][1], self.plan.stn.getBounds(str(a["tStart"])).lb, self.plan.stn.getBounds(str(a["tEnd"])).lb) for a in self.plan.actions if a["name"] != "dummy init"]
        info.sort(key=lambda x: x[2])
        
        for i in info:
            #print "%s (%s) : [%f, %f]" % i
            print("{: <70} ({}) : [{}, {}]".format(*i))

    def update(self):
        if not self.state == State.RUNNING:
            return # do not execute the plan if not running

        l = next(self.getExecutableTps(), False)

        while l:
            logger.debug("Next executable tp : %s" % (str(l)))
            logger.debug("Current time : %d" % self.getCurrentTime())
            logger.debug("Bounds : %s" % self.plan.stn.getBounds(l))
            self.executeTp(l)
            l = next(self.getExecutableTps(), False)
                    
        if not self.plan.stn.isConsistent():
            logger.error("Execution of the plan when executing controllable points")
            raise ExecutionFailed("Execution of the plan when executing controllable points")

    #Called when an alea is received
    def dealAlea(self, aleaType, data):
        if aleaType == "robotDead":
            if "robot" not in data:
                logger.error("Missing field robot in an alea of type robotDead")
                return
            
            if data["robot"] == self.agent:
                logger.warning("Received a robotDead for myself. Deactivating")
                self.state = State.DEAD
                return
            else:
                logger.warning("Dealing with the death of %s" % data["robot"])
                self.agentsDead.append(data["robot"])
                raise ExecutionFailed("Received an alea of type robotDead")
        elif aleaType == "targetFound":
            targetPos = data.get("position", None)
            self.targetFound(targetPos)
        else:
            logger.error("Cannot deal with an alea of unknown type : %s" % aleaType)
            return


    # Called when the STN is updated. Used to send data back to the mission center
    def stnUpdated(self, onlyPast = False):
        pass

    #type can be repairRequest, repairDone, targetFound
    # repairRequest : no data
    # repairDone    : data is a string : the new plan
    # targetFound   : data is a dict with keys x,y for the targetpos
    def sendNewStatusMessage(self, t, data = None):
        pass
    
    def repairCallback(self, t, time, sender, msg):
        pass

    def computeGlobalPlan(self):
        self.repairResponse = {self.agent:self.plan.getLocalJsonPlan(self.agent)}
        
        if self.repairRos:
            self.sendNewStatusMessage("repairRequest")
            
            time.sleep(2)
            
            logger.info("Receive responses from : %s" % str(self.repairResponse.keys()))
            logger.info("Dead agents : %s" % self.agentsDead)
            for agent in self.agentsDead:
                if agent in self.repairResponse:
                    logger.warning("Received a repair response from %s. Ignoring it since he is dead" % agent)
                    del self.repairResponse[agent]
        else:
            pass
        
        #lock all the steps of agents that did not respond
        allAgents = set([a["agent"] for a in self.plan.actions.values() if "agent" in a])
        logger.info("All agents : %s" % allAgents)
        
        for agent in allAgents:
            if agent in self.repairResponse:
                continue #
            
            logger.info("Locally adding plan for %s" % agent)
            
            localPlan = self.plan.getLocalJsonPlan(agent)
            
            if agent not in self.agentsDead:
                for a in localPlan["actions"].values():
                    a["locked"] = True
            self.repairResponse[agent] = copy(localPlan)
        
        
        planJson = plan.Plan.mergeJsonPlans(self.repairResponse)
        planJson["current-time"] = time.time() - self.beginDate

        #Remove coordinating action for dead robots
        deletedActionKeys = set()
        deletedTps = set()
        
        for k,a in planJson["actions"].items():
            if "communicate-meta" in a["name"]:
                for deadAgent in self.agentsDead:
                    if deadAgent in a["name"]:
                        deletedActionKeys.add(k)
                        deletedTps.add(a["startTp"])
                        deletedTps.add(a["endTp"])
                        break

        for k in deletedActionKeys:
            logger.info("Removing action %s" % planJson["actions"][k]["name"])
            del planJson["actions"][k]
        for i in reversed(range(len(planJson["causal-links"]))):
            cl = planJson["causal-links"][i]
            if cl["startTp"] in deletedTps or cl["endTp"] in deletedTps:
                del planJson["causal-links"][i]
        for i in reversed(range(len(planJson["temporal-links"]))):
            cl = planJson["temporal-links"][i]
            if cl["startTp"] in deletedTps or cl["endTp"] in deletedTps:
                del planJson["temporal-links"][i]
        for i in reversed(range(len(planJson["absolute-time"]))):
            if planJson["absolute-time"][i][0] in deletedTps:
                del planJson["absolute-time"][i]

        return planJson
        
    def repairPlan(self, planJson):
    
        if self.pddlFiles is None:
            logger.error("No provided PDDL files. Cannot repair")
            return None
        
        with open("plan-broken.plan", "w") as f:
            json.dump(planJson, f)
        
        #compute the list of available agents
        agents = set(self.repairResponse.keys())
        agents.add(self.agent)
        
        if len(self.agentsDead) > 0 :
            for a in self.agentsDead:
                if a in agents:
                    agents.remove(a)

        #Write the content of the pddl file to disk since hipop can only read files
        with open("plan-broken-domain.pddl", "w") as f:
            f.write(self.pddlFiles["domain"])

        with open("plan-broken-prb.pddl", "w") as f:
            f.write(self.pddlFiles["prb"])

        with open("plan-broken-helper.pddl", "w") as f:
            f.write(self.pddlFiles["helper"])

        outputFile = tempfile.NamedTemporaryFile("w+")
        
        command = "hipop -L error -u --timing -H plan-broken-helper.pddl -I plan-broken.plan --agents {agents} -P hadd_time_lifo -A areuse_motion_nocostmotion -F local_openEarliestMostCostFirst_motionLast -O plan-repaired.pddl -o plan-repaired.plan plan-broken-domain.pddl plan-broken-prb.pddl".format(agents="_".join(agents))
        logger.info("Launching hipop with %s" % command)
        try:
            r = call(command.split(" "), stdout=outputFile, stderr= subprocess.STDOUT, timeout = 30)
        except OSError as e:
            if e.errno == os.errno.ENOENT:
                # handle file not found error.
                logger.error("Hipop is not found. Install it to repair.")
                return None
            else:
                # Something else went wrong while trying to run the command
                logger.error("During the reparation something went wrong. Returned %s. Cannot repair." % r)
                outputFile.seek(0)
                for l in outputFile.readlines():
                    logger.error(l.replace("\n",""))
                return None

        except subprocess.TimeoutExpired as e:
            logger.error("During reparation : hipop timeout")
            return None
        
        if(r == 0):
            with open("plan-repaired.plan") as f:
                planStr = " ".join(f.readlines())
        
            logger.info("reparation succes")
            return planStr

        else:
            logger.error("During the reparation hipop returned %s. Cannot repair." % r)
            if r == 1:
                logger.error("No solution was found by hipop")
            outputFile.seek(0)
            for l in outputFile.readlines():
                logger.error(l.replace("\n",""))
            return None

    
    def executionFail(self):
        
        self.state = State.REPAIRINGACTIVE
        
        planJson = self.computeGlobalPlan()
        
        #Assume failure because of a deadline
        #Find the next deadline and if possible (action not locked) iteratively shift it
        """
        if not self.plan.stn.isConsistent():
            logger.info("Trying to shift deadlines")
            comMetaKeys = [k for k,a in planJson["actions"].items() if "communicate-meta" in a["name"] and self.agent in a["name"] and ("locked" not in a or not a["locked"])]
            
            if len(comMetaKeys) == 0:
                logger.error("Failed because of a deadline : no communicate-meta action to shift")
                logger.error(planJson["actions"])
                self.state = State.ERROR
                self.stnUpdated()
                sys.exit(1)
            else:
                logger.info("I can shift : %s" % [planJson["actions"][k]["name"] for k in comMetaKeys])
            
            tps = set(itertools.chain.from_iterable((planJson["actions"][k]["startTp"], planJson["actions"][k]["endTp"]) for k in comMetaKeys))
            logger.info("I can move tps : %s" % tps)
            
            for i in range(5): #number of repair tries
                shift = 10 # in seconds
                for i in range(len(planJson["absolute-time"])):
                    tp,value = planJson["absolute-time"][i]
                    if tp in tps:
                        planJson["absolute-time"][i] = [tp, value + shift]
                
                planStr = self.sendNewStatusMessage("repairDone", json.dumps(planJson))
                if planStr is not None:
                    break

        else:
        """
        planStr = self.repairPlan(planJson)
        
        if planStr is None:
            logger.warning("Reparation failed. Trying to remove deadlines.")
            
            #Try to remove the furure deadlines
            futureComs = []
            for a in self.plan.actions.values():
                if "communicate-meta" in a["name"] and self.agent in a["name"] and not a["executed"]:
                    futureComs.append(a["name"])
                    
            if len(futureComs) > 0:
                logger.info("Found some coms to drop : %s" % futureComs)
                for c in futureComs:
                    self.dropCommunication(c)
                    
                planStr = self.repairPlan(planJson)
                
                if planStr is None:
                    logger.error("Even after dropping communication, cannot repair")
                    logger.error(self.plan.stn.export())
                    #TODO : implement a default strategy ? Notify other agents ?
                    self.state = State.ERROR
                    self.stnUpdated()
                    sys.exit(1)
            else:
                logger.error("Found no deadlines to remove. Cannot repair")
                #TODO : implement a default strategy ? Notify other agents ?
                self.state = State.ERROR
                self.stnUpdated()
                sys.exit(1)

        #We have a new plan
        self.sendNewStatusMessage("repairDone", planStr)
        del self.repairResponse

        self.init(planStr, self.agent)
        logger.info("Finished reparation : restarting the main loop")
        self.state = State.RUNNING
        self.mainLoop()
        
    def run(self):
        logger.info("Supervisor launched")
        
        self.startEvent.wait()

        self.state = State.RUNNING

        if self.plan.initTime is None:
            #begining of the plan
            self.beginDate = time.time()
        else:
            logger.info("Executing an half-executed plan. Starting at %s" % self.plan.initTime)
            self.beginDate = time.time() - self.plan.initTime

        #executing start dummy action
        self.executedTp["0"] = 0
        self.tp["0-start-dummy init"] = ["dummy init", "past"]
        
        self.outQueue.put({"type":"start", "startTime":self.beginDate})
        #self.outQueue.put({"type":"startAction", "action":{"name" : "move", "dMin" : 1}})
        self.mainLoop()

    def mainLoop(self):
        hasFailed = False
        try:
            while not self.isExecuted() and self.state != State.DEAD and not self.stopEvent.is_set():
                with self.mutex:
                    if self.state == State.RUNNING:
                        while not self.inQueue.empty():
                            msg = self.inQueue.get()
            
                            if not isinstance(msg, dict) or "type" not in msg:
                                logger.error("Supervisor received an ill-formated message : %s" % msg)
                                continue
                            
                            elif msg["type"] == "endAction":
                                self.endAction(msg)
                            elif msg["type"] == "alea":
                                self.dealAlea(msg.get("aleaType"), msg)
                            elif msg["type"] == "ubAction":
                                self.checkActionUB(msg)
                            else:
                                logger.warning("Supervisor received unknown message %s" % msg)
                            
                    if self.state != State.DEAD:
                        self.update()

                self.stopEvent.wait(0.1)
        except ExecutionFailed as e:
            hasFailed = True
            logger.error("Execution failed : %s" % str(e))
            
        if hasFailed:
            self.executionFail()
        else:
            self.outQueue.put({"type":"stop"})
            logger.info("Supervisor dead")
            logger.info("Is stop event set ? : %s" % self.stopEvent.is_set())
            logger.info("Is executed ? : %s" % self.isExecuted())
            logger.info("state ? : %s" % self.state)
