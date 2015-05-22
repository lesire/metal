from __future__ import division


from copy import copy
from math import floor
import json
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

State = enum("INIT", "RUNNING", "REPAIRINGACTIVE", "REPAIRINGPASSIVE", "TRACKING", "DEAD")

class ExecutionFailed(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

class Supervisor(threading.Thread):
    def __init__ (self, inQueue, outQueue, planStr, startEvent, stopEvent, agent = None, pddlFiles=None, useMaSTN = False):
        self.inQueue = inQueue
        self.outQueue = outQueue
        self.pddlFiles = pddlFiles
        
        self.beginDate = -1
        self.state = State.INIT
        self.repairRos = False
        self.useMaSTN = useMaSTN

        self.agentsDead = []

        self.startEvent = startEvent
        self.stopEvent = stopEvent

        threading.Thread.__init__ (self, name="%s-sup" % agent)
        
        self.init(planStr, agent)
        
    
        
    def init(self, planStr, agent):
        if agent is None:
            self.plan = plan.Plan(planStr)
        else:
            self.plan = plan.Plan(planStr, agent)
        self.agent = agent
        
        self.executedTp = {}
        self.tp = {}
        
        if self.useMaSTN:
            logger.info("Using MaSTN")
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
            elif self.useMaSTN and a["agent"] != self.agent:
                if a["tStart"] in self.plan.stn.getNodeIds():
                    self.tp[a["tStart"]] = [a["name"], "uncontrollable"]
                if a["tEnd"] in self.plan.stn.getNodeIds():
                    self.tp[a["tEnd"]] = [a["name"], "uncontrollable"]
            elif "abstract" in a:
                self.tp[a["tStart"]] = [a["name"], "controllable"]
                self.tp[a["tEnd"]] =   [a["name"], "controllable"]
            elif a["controllable"]:
                self.tp[a["tStart"]] = [a["name"], "controllable"]
                self.tp[a["tEnd"]] =   [a["name"], "controllable"]
            else:
                self.tp[a["tStart"]] = [a["name"], "controllable"]
                self.tp[a["tEnd"]] =   [a["name"], "uncontrollable"]
        
        for tpName,value in self.plan.absTimes:
            action = [a for a in self.plan.actions.values() if (a["tStart"] == tpName or a["tEnd"] == tpName) and "dummy" not in a["name"]][0]
            if action["executed"]:
                #action was executed, this is a past action
                self.executedTp[tpName] = value
                self.tp[tpName][1] = "past"
            else:
                self.tp[tpName][1] = "future"

            self.plan.stn.addConstraint(self.plan.stn.getStartId(), tpName, value, value)
            
            if not self.plan.stn.isConsistent():
                logger.error("Error : Invalid stn when setting the time of an absolute tp : %s" % tpName)
                raise ExecutionFailed("Invalid stn when setting the time of an absolute tp : %s" % tpName)

            #TODO if this is an action being executed, send a message to the executor
    
        for tpName,value in self.plan.absTimes:
            action = [a for a in self.plan.actions.values() if (a["tStart"] == tpName or a["tEnd"] == tpName) and "dummy" not in a["name"]][0]
                
            if action["executed"] and action["tStart"] == tpName and not self.isResponsibleForAction(action) and self.tp[action["tEnd"]][1] != "past":
                logger.info("When importing, setting the end time of %s" % action["name"])
                self.tp[action["tEnd"]][1] = "future"
                if "abstract" in action:
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
        #Retrieve the corresponding action
        a = [a for a in self.plan.actions.values() if (a["tStart"] == tp or a["tEnd"] == tp)][0]
        logger.debug("Executing tp : %s" % tp)
        
        
        if self.tp[tp][1] == "future":
            #If this is a deadline, must use the exact deadline time
            currentTime = self.plan.stn.getBounds(tp).lb
        elif a["tEnd"] == tp and self.tp[tp][1] == "controllable":
            #If this is the end of a action with strict duration
            currentTime = self.plan.stn.getBounds(tp).lb
        else:
            currentTime = self.getCurrentTime()

        if a["tStart"] == tp:
            self.executeAction(a, currentTime)
        elif a["tEnd"] == tp:

            if self.tp[tp][1] == "controllable":
                #End of a controllable action
                self.setTimePoint(tp, currentTime)
                
                if not self.plan.stn.isConsistent():
                    logger.warning("\tError : invalid STN when finishing execution of %s" % a["name"])
                    raise ExecutionFailed("\tInvalid STN when finishing execution of %s" % a["name"])

                if not "abstract" in a and self.isResponsibleForAction(a):
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

        self.executedTp[action["tStart"]] = currentTime
        self.tp[action["tStart"]][1] = "past"
        
        self.setTimePoint(action["tStart"], currentTime)

        if action["name"] == "dummy end":
            logger.info("finished the plan")
            return

        if "abstract" not in action:
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

    def targetFound(self, targetPos = None):
        self.state = State.TRACKING
        self.outQueue.put({"type":"startAction", "action":{"name":"track", "dMin":1}, "time":self.getCurrentTime()})
    
    def endAction(self, msg):
        action = msg["action"]
        tp = action["tEnd"]
        value = msg["time"]

        report = msg.get("report", None)
        
        if isinstance(report, str):
            report = {"type" : report}

        if report is None:
            logger.warning("End of action %s without report" % action["name"])
        elif report["type"] == "ok":
            logger.info("End of action %s ok" % action["name"])
        else:
            logger.warning("End of action %s with status %s" % (action["name"], report))
            if "target" in report["type"]:
                targetPos = report.get("position", None)
                self.targetFound(targetPos)
        
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
        dMin = action["dMin"]*plan.timeFactor
        if dReal < dMin:
            logger.warning("An action finished early : %.2f instead of %.2f. Updating its dMin." % (dReal/1000, dMin/1000))
            self.plan.stn.setConstraint(action["tStart"], tp, int(floor(dReal)))
        
        lb = self.plan.stn.getBounds(tp).lb
        if self.tp[tp][1] == "uncontrollable" and value < lb:
            logger.warning("Finished %s early : %s. Waiting until %s." % (action["name"], value, lb))
            value = lb
            
        logger.info("End of the action %s at %s" % (action["name"], value/1000))
        
        c = self.plan.stn.getBounds(str(tp))
        
        logger.debug("Is STN Consistent : %s" % self.plan.stn.isConsistent())
        logger.debug("Bounds %s" % c)
        logger.debug("May be consistent %s " % self.plan.stn.mayBeConsistent(self.plan.stn.getStartId(), str(tp), value, value))
            
        self.setTimePoint(tp, value)
            
        if not self.plan.stn.isConsistent():
            logger.error("\tError : invalid STN when finishing execution of tp %s" % tp)
            raise ExecutionFailed("Invalid STN when finishing execution of tp %s" % tp)

            
        self.executedTp[tp] = value
        self.tp[tp][1] = "past"

    def isExecuted(self):
        return all([tp[1] == "past" for tp in self.tp.values()])
        
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
        else:
            logger.error("Cannot deal with an alea of unknown type : %s" % aleaType)
            return


    # Called when the STN is updated. Used to send data back to the mission center
    def stnUpdated(self, onlyPast = False):
        pass

    #if data = None : repair request
    #if data is a str : assume this is a new repaired plan
    def sendRepairMessage(self, data = None):
        pass
    
    def repairCallback(self, type, time, sender, msg):
        pass

    def executionFail(self):
        
        self.state = State.REPAIRINGACTIVE
        self.stnUpdated()
        
        self.repairResponse = {self.agent:self.plan.getLocalJsonPlan(self.agent)}
        
        if self.repairRos:
            self.sendRepairMessage()
            
            time.sleep(2)
            
            logger.info("Receive responses from : %s" % str(self.repairResponse.keys()))
            logger.info("Dead agents : %s" % self.agentsDead)
            for agent in self.agentsDead:
                logger.info("%s %s %s" % (agent in self.repairResponse, "mana" in self.repairResponse, agent))
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
        
        #Failure because of a deadline : remove the next one
        if not self.plan.stn.isConsistent():
            for i in reversed(range(len(planJson["absolute-time"]))):
                tp,value = planJson["absolute-time"][i]
                if value*1000 > self.getCurrentTime():
                    #in the future : this is a deadline
                    l = [ ("communicate-meta" in a["name"] and self.agent in a["name"]) for a in planJson["actions"].values() if a["startTp"] == tp or a["endTp"] == tp]
                    if any(l):
                        logger.info("Deleting %s,%s from the plan to remove a deadline" %(tp,value))
                        del planJson["absolute-time"][i]
        
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
        
        with open("plan-broken.plan", "w") as f:
            json.dump(planJson, f)
        
        if self.pddlFiles is None:
            logger.error("No provided PDDL files. Cannot repair")
            sys.exit(1)
            
        #compute the list of available agents
        agents = set(self.repairResponse.keys())
        agents.add(self.agent)
        
        if len(self.agentsDead) > 0 :
            for a in self.agentsDead:
                if a in agents:
                    agents.remove(a)
        
        del self.repairResponse
                
        #Write the content of the pddl file to disk since hipop can only read files
        domainFile = tempfile.NamedTemporaryFile("w")
        domainFile.write(self.pddlFiles["domain"])
        domainFile.flush()
        
        prbFile = tempfile.NamedTemporaryFile("w")
        prbFile.write(self.pddlFiles["prb"])
        prbFile.flush()
        
        helperFile = tempfile.NamedTemporaryFile("w")
        helperFile.write(self.pddlFiles["helper"])
        helperFile.flush()
        
        command = "hipop -L error --timing -H {helper} -I plan-broken.plan --agents {agents} -P hadd_time_lifo -A areuse_motion_nocostmotion -F local_openEarliestMostCostFirst_motionLast -O plan-repaired.pddl -o plan-repaired.plan {domain} {prb}".format(domain=domainFile.name, prb=prbFile.name, helper=helperFile.name, agents="_".join(agents))
        logger.info("Launching hipop with %s" % command)
        try:
            r = subprocess.call(command.split(" "))
        except OSError as e:
            if e.errno == os.errno.ENOENT:
                # handle file not found error.
                logger.error("Hipop is not found. Install it to repair.")
                sys.exit(1)
            else:
                # Something else went wrong while trying to run the command
                logger.error("During the reparation hipop returned %s. Cannot repair." % r)
                sys.exit(1)
        
        if(r == 0):
            with open("plan-repaired.plan") as f:
                planStr = " ".join(f.readlines())
            
            #We have a new plan
            self.sendRepairMessage(planStr)
            
            self.init(planStr, self.agent)
            logger.info("Finished repairation : restarting the main loop")
            self.state = State.RUNNING
            self.mainLoop()
        else:
            logger.error("During the reparation hipop returned %s. Cannot repair." % r)
            if r == 1:
                logger.error("No solution was found by hipop")
            sys.exit(1)
                

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
