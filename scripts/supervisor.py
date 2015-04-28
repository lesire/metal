from __future__ import division

try:
    import Queue
    version = 2
except:
    import queue as Queue
    version = 3

from copy import copy
import json
import os
import subprocess
import sys
import threading
import time

import logging; logger = logging.getLogger("hidden")

import plan

class ExecutionFailed(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

class Supervisor(threading.Thread):
    def __init__ (self, inQueue, outQueue, planStr, agent = None, pddlFiles=None):
        self.inQueue = inQueue
        self.outQueue = outQueue
        self.pddlFiles = pddlFiles
        
        self.beginDate = -1
        self.repairRos = False

        self.init(planStr, agent)
        
        threading.Thread.__init__ (self, name="Supervisor %s" % agent)
        
    def init(self, planStr, agent):
        if agent is None:
            self.plan = plan.Plan(planStr)
        else:
            self.plan = plan.Plan(planStr, agent)
        self.agent = agent
        
        self.executedTp = {}
        self.tp = {}
        
        for a in self.plan.actions.values():
            if a["name"] == "dummy init":
                self.tp[a["tStart"]] = [a["name"], "uncontrollable"]
            elif a["name"] == "dummy end":
                self.tp[a["tStart"]] = [a["name"], "controllable"]
            elif ("dummy" in a["name"]):
                pass
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

            #TODO if this is an action beeing executed, send a message to the executor
    
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
                self.plan.setTimePoint(tp, currentTime)
                
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
        
        self.stnUpdated()

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
        
        self.plan.setTimePoint(action["tStart"], currentTime)

        if action["name"] == "dummy end":
            logger.info("finished the plan")
            return

        if "abstract" not in action:
            if not self.isResponsibleForAction(action):
                #This action should not be executed by this robot. Assume someone else will do it
                self.tp[action["tEnd"]][1] = "future"
                if action["controllable"]:
                    self.plan.setTimePoint(action["tEnd"], self.plan.stn.getBounds(action["tEnd"]).lb)
                else:
                    self.plan.setTimePoint(action["tEnd"], currentTime + int(round(1000 * action["dMin"])))

                logger.debug("Action %s is not for this agent. Assume it will last is minimum duration" % action["name"])
            else:
                #send execution order
                msg = {"type":"startAction", "action":copy(action), "time":currentTime}
                
                logger.debug("Sending message : %s" % msg)
                self.outQueue.put(msg)
        
        if not self.plan.stn.isConsistent():
            logger.error("\tError : invalid STN when launching execution of %s" % action["name"])
            raise ExecutionFailed("Invalid STN when launching execution of %s" % action["name"])

    def endAction(self, msg):
        action = msg["action"]
        tp = action["tEnd"]
        value = msg["time"]

        report = msg.get("report", None)

        if report is None:
            logger.warning("End of action %s without report" % action["name"])
        elif report == "ok":
            logger.info("End of action %s ok" % action["name"])
        else:
            logger.warning("End of action %s with status" % (action["name"], report))

        if action["controllable"]:
            logger.info("Notified of the end of controllable action %s." % action["name"])
            if self.tp[tp][1] != "past":
                logger.error("Notified of the end of controllable action %s." % action["name"])
                logger.error("But the timepoint is not past : %s" % self.tp[tp][1])
            return
        
        lb = self.plan.stn.getBounds(tp).lb
        if self.tp[tp][1] == "uncontrollable" and value < lb:
            logger.warning("Finished %s early : %s. Waiting until %s." % (action["name"], value, lb))
            value = lb
            
        logger.info("End of the action %s at %s" % (action["name"], value/1000))
        
        c = self.plan.stn.getBounds(str(tp))
        
        logger.debug("Is STN Consistent : %s" % self.plan.stn.isConsistent())
        logger.debug("Bounds %s" % c)
        logger.debug("May be consistent %s " % self.plan.stn.mayBeConsistent(self.plan.stn.getStartId(), str(tp), value, value))
            
        self.plan.setTimePoint(tp, value)
            
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

    # Called when the STN is updated. Used to send data back to the mission center
    def stnUpdated(self):
        pass

    def sendRepairMessage(self, msg):
        pass
    
    def repairCallback(self, msg):
        pass

    def executionFail(self):
        planJson = self.plan.getJsonDescription()
        planJson["current-time"] = time.time() - self.beginDate
        
        if self.repairRos:
            self.repairResponse = {}
            msg = {"agent":self.agent, "type":"repairRequest"}
            msg = json.dumps(msg)
            self.sendRepairMessage(msg)
            
            time.sleep(5)
            
            logger.info("Receive responses from : %s" % str(self.repairResponse.keys()))
            
            for agent,plan in self.repairResponse:
                for k,a in plan["actions"].items():
                    if "locked" in a and a["locked"]:
                        if k in planJson["actions"] and a["name"] == planJson["actions"][k]["name"]:
                            planJson["actions"][k]["locked"] = True
                        else:
                            #locked action that I did not know about
                            logger.warning("%s has a locked action that I did not know about %s %s: " % (agent,k,a["name"]))
                            logger.warning("%s" % str(a))
                            if k in planJson["actions"]:
                                logger.warning(planJson["actions"][k])
            
        else:
            """
            for a in planJson["actions"].values():
                if not self.isResponsibleForAction(a):
                    a["locked"] = True
            """
            pass
        
        #remove all deadlines for the reparation
        logger.info("Removing all deadlines from the plan")
        planJson["absolute-time"] = list(filter(lambda d: d[1] <= planJson["current-time"], planJson["absolute-time"]))
        
        with open("plan-broken.plan", "w") as f:
            json.dump(planJson, f)
        
        if self.pddlFiles is None:
            logger.error("No provided PDDL files. Cannot repair")
            sys.exit(1)
        
        command = "hipop -L error --timing -H {helper} -I plan-broken.plan -P hadd_time_lifo -A areuse_motion_nocostmotion -F local_openEarliestMostCostFirst_motionLast -O plan-repaired.pddl -o plan-repaired.plan {domain} {prb}".format(**self.pddlFiles)
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
            self.init(planStr, self.agent)
            self.mainLoop()
        else:
            logger.error("During the reparation hipop returned %s. Cannot repair." % r)
            if r == 1:
                logger.error("No solution was found by hipop")
            sys.exit(1)
                

    def run(self):
        logger.info("Supervisor launched")
        
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
            while not self.isExecuted():
                if not self.inQueue.empty():
                    msg = self.inQueue.get()
    
                    if type(msg) != dict or "type" not in msg:
                        logger.error("Supervisor received an ill-formated message : %s" % msg)
                        continue
                    
                    elif msg["type"] == "endAction":
                        self.endAction(msg)
                    else:
                        logger.warning("Supervisor received unknown message %s" % msg)
                self.update()
                time.sleep(0.1)
        except ExecutionFailed as e:
            hasFailed = True
            logger.error("Execution failed")
            logger.error(str(e))
            
        if hasFailed:
            self.executionFail()
        else:
            self.outQueue.put({"type":"stop"})
            logger.info("Supervisor dead")
