from __future__ import division

try:
    import Queue
    version = 2
except:
    import queue as Queue
    version = 3

from copy import copy
import logging
import threading
import time

import plan

class ExecutionFailed(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

class Supervisor(threading.Thread):
    def __init__ (self, inQueue, outQueue, planStr, agent = None):
        self.inQueue = inQueue
        self.outQueue = outQueue
        self.plan = plan.Plan(planStr)
        self.agent = agent
        
        self.executedTp = {}
        self.tp = {}
        self.beginDate = -1
        
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
        
        j = self.plan.getJsonDescription()
        if "absolute-time" in j and j["absolute-time"]:
            for tp,value in j["absolute-time"]:
                
                #find the timepoint name
                tpCandidate = filter(lambda x: x.startswith(str(tp) + "-"), self.tp.keys())
                try:
                    tpName = next(tpCandidate)
                except StopIteration:
                    logging.error("Error : Cannot find an exected tp %s" % tp)
                    raise ExecutionFailed("Cannot find an exected tp")
                
                value = int(round(1000*value))
                
                action = [a for a in self.plan.actions.values() if (a["tStart"] == tpName or a["tEnd"] == tpName) and "dummy" not in a["name"]][0]
                if action["executed"]:
                    #action was executed, this is a past action
                    self.executedTp[tpName] = value
                    self.tp[tpName][1] = "past"
                else:
                    self.tp[tpName][1] = "future"

                self.plan.stn.addConstraint(self.plan.stn.getStartId(), tpName, value, value)
                
                if not self.plan.stn.isConsistent():
                    logging.error("Error : Invalid stn when setting the time of an absolute tp : %s" % tpName)
                    raise ExecutionFailed("Invalid stn when setting the time of an absolute tp : %s" % tpName)

                #TODO if this is an action beeing executed, send a message to the executor
                
        threading.Thread.__init__ (self, name="Supervisor")
    
    def getExecutableTps(self, now = True):
        return filter(lambda tp: self.isTpExecutable(tp, now), self.tp.keys())
    
    def getCurrentTime(self):
        if self.beginDate < 0:
            logging.error("getCurrentTime called before initialisation")

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
        elif self.agent in action["name"]:
            return True
        
        return False
    
    def executeTp(self, tp):
        #Retrieve the corresponding action
        a = [a for a in self.plan.actions.values() if (a["tStart"] == tp or a["tEnd"] == tp)][0]
        
        #If this is a deadline, must use the exact deadline time
        if self.tp[tp][1] == "future":
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
                    logging.warning("\tError : invalid STN when finishing execution of %s" % a["name"])
                    raise ExecutionFailed("\tInvalid STN when finishing execution of %s" % a["name"])

                if not "abstract" in a and self.isResponsibleForAction(a):
                    logging.info("Stop of action {a} at time {t}".format(a=a["name"],t=currentTime))

                    msg = {"type":"stopAction", "action":copy(a), "time":currentTime}
                    self.outQueue.put(msg)
                    
                
            self.tp[tp][1] = "past"
            self.executedTp[tp] = currentTime
                
        else:
            logging.error("\tError : a timepoint does not match its action %s" % tp)
            raise ExecutionFailed("\tA timepoint does not match its action %s" % tp)


    def executeAction(self, action, currentTime):
        if self.tp[action["tStart"]][1] != "controllable" and self.tp[action["tStart"]][1] != "future":
            logging.error("Cannot execute %s, the status of its start point is %s." % (action["name"], self.tp[action["tStart"]] ))
            return

        if self.isResponsibleForAction(action):
            logging.info("Starting %s at time %f." % (action["name"], currentTime/1000))

        self.executedTp[action["tStart"]] = currentTime
        self.tp[action["tStart"]][1] = "past"
        
        self.plan.setTimePoint(action["tStart"], currentTime)

        if action["name"] == "dummy end":
            logging.info("finished the plan")
            return

        if "abstract" not in action:
            if not self.isResponsibleForAction(action):
                #This action should not be executed by this robot. Assume someone else will do it
                self.tp[action["tEnd"]][1] = "future"
                if action["controllable"]:
                    self.plan.setTimePoint(action["tEnd"], self.plan.stn.getBounds(action["tEnd"]).lb)
                else:
                    self.plan.setTimePoint(action["tEnd"], currentTime + int(round(1000 * action["dMin"])))

                logging.debug("Action %s is not for this agent. Assume it will last is minimum duration" % action["name"])
            else:
                #send execution order
                msg = {"type":"startAction", "action":copy(action), "time":currentTime}
                
                logging.debug("Sending message : %s" % msg)
                self.outQueue.put(msg)
        
        if not self.plan.stn.isConsistent():
            logging.error("\tError : invalid STN when launching execution of %s" % action["name"])
            raise ExecutionFailed("Invalid STN when launching execution of %s" % action["name"])

    def endAction(self, msg):
        action = msg["action"]
        tp = action["tEnd"]
        value = msg["time"]
        
        if action["controllable"]:
            logging.info("Notified of the end of controllable action %s." % action["name"])
            if self.tp[tp][1] != "past":
                logging.error("Notified of the end of controllable action %s." % action["name"])
                logging.error("But the timepoint is not past : %s" % self.tp[tp][1])
            return
        
        lb = self.plan.stn.getBounds(tp).lb
        if self.tp[tp][1] == "uncontrollable" and value < lb:
            logging.warning("Finished %s early : %s. Waiting until %s." % (action["name"], value, lb))
            value = lb
            
        logging.info("End of the action %s at %s" % (action["name"], value))
        
        c = self.plan.stn.getBounds(str(tp))
        
        logging.debug("Is STN Consistent : %s" % self.plan.stn.isConsistent())
        logging.debug("Bounds %s" % c)
        logging.debug("May be consistent %s " % self.plan.stn.mayBeConsistent(self.plan.stn.getStartId(), str(tp), value, value))
            
        self.plan.setTimePoint(tp, value)
            
        if not self.plan.stn.isConsistent():
            logging.error("\tError : invalid STN when finishing execution of tp %s" % tp)
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
            logging.debug("Next executable tp : %s" % (str(l)))
            self.executeTp(l)
            l = next(self.getExecutableTps(), False)
                    
        if not self.plan.stn.isConsistent():
            logging.error("Execution of the plan when executing controllable points")
            raise ExecutionFailed("Execution of the plan when executing controllable points")

    def run(self):
        logging.info("Supervisor launched")
        self.beginDate = time.time()
        
        #executing start dummy action
        self.executedTp["0"] = 0
        self.tp["0-start-dummy init"] = ["dummy init", "past"]
        
        self.outQueue.put({"type":"start", "startTime":self.beginDate})
        #self.outQueue.put({"type":"startAction", "action":{"name" : "move", "dMin" : 1}})

        while not self.isExecuted():
            if not self.inQueue.empty():
                msg = self.inQueue.get()

                if type(msg) != dict or "type" not in msg:
                    logging.error("Supervisor received an ill-formated message : %s" % msg)
                    continue
                
                elif msg["type"] == "endAction":
                    self.endAction(msg)
                else:
                    logging.warning("Supervisor received unknown message %s" % msg)
            self.update()
            time.sleep(0.1)
            
        self.outQueue.put({"type":"stop"})
        logging.info("Supervisor dead")
