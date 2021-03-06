from __future__ import division

import logging; logger = logging.getLogger("hidden")

import threading
import time

class Executor(threading.Thread):
    """Abstract call that represents an executor.
    
    Its goal is to run to do the interface between the supervisor and the autonomous agent.
    It receives orders from the supervisor (start/stop and action) and send status report to the supervisor (action done, alea received, etc ...)
    
    If not overriden by subclasses, it will simulate the execution of all actions (ie. wait the their nominal time before signaling their end)
    """

    def __init__ (self, inQueue, outQueue, stopEvent, actionExecutor=None, agent=None):
        self.inQueue = inQueue
        self.outQueue = outQueue
        
        self.actionExecutor = actionExecutor
        self.actionExecutor.outQueue = self.outQueue #shate the outque to allow the actionExecutor to write aleas

        self.isStarted = False
        self.nextEvents = [] #list of dicts (time, action, report (opt))
        self.beginDate = -1
        threading.Thread.__init__ (self, name="%s-exec" % agent)
        self.stopEvent = stopEvent

    # get readable time
    def user_time(self, t):
        """convert the time return by time.time (is seconds since epoch) to the time used in the plan (ms since the begining of the mission)"""
        return int(round(1000*(t-self.beginDate)))

    # end action callback
    def action_callback(self, action, report=None):
        currentTime = time.time()
        self.nextEvents.append({"time":currentTime, "action":action, "report":report})
        logger.info("Callback of action {a} at time {t} with report {r}".format(a=action["name"],t=self.user_time(currentTime)/1000, r=report))

    #called periodically
    def update(self):
        currentTime = time.time()
        
        self.actionExecutor.update()
        
        for action,report in [(d["action"],d.get("report", None)) for d in self.nextEvents if d["time"] <= currentTime]:
            self.outQueue.put({"type":"endAction", "action":action, "time": self.user_time(currentTime), "report":report})
            logger.info("End of action %s with report %s" % (action["name"], report))
            logger.debug("End of action %s" % action)
            
        self.nextEvents = [d for d in self.nextEvents if d["time"] > currentTime]
        
    def startAction(self, msg):
        if "action" not in msg or type(msg["action"]) != dict:
            logger.error("Executor received an ill formated action : %s" % msg)
            return
        
        action = msg["action"]
        currentTime = time.time()
        logger.info("Start of action {a} at time {t}".format(a=action["name"],t=self.user_time(currentTime)/1000))

        try:
            self.actionExecutor.execute(action, self.action_callback, time=self.user_time(currentTime))
        except AttributeError as e:
            logger.error("Cannot execute %s." % action["name"])
            logger.error(e)
            self.action_callback(msg["action"])
            pass

    def stopAction(self, msg):
        action = msg["action"]
        currentTime = time.time()
        logger.info("Stop of action {a} at time {t}".format(a=action["name"],t=self.user_time(currentTime)))
        self.actionExecutor.stop(action)

    def startExecutor(self, msg):
        logger.info("Executor received start message")
        self.isStarted = True
        self.beginDate = msg["startTime"]
        
    def stopExecutor(self, msg):
        logger.info("Executor received stop message")
        self.actionExecutor.stopExecutor(self.user_time(time.time()))

    def run(self):
        logger.info("Executor launched")
        
        while not self.stopEvent.is_set():
            if not self.inQueue.empty():
                msg = self.inQueue.get()
                logger.debug("Executor received message : %s" % msg)

                
                if type(msg) != dict or "type" not in msg:
                    logger.error("Executor received an ill-formated message : %s" % msg)
                    continue
                
                if not self.isStarted and msg["type"] != "start":
                    logger.error("Executor received a message before its start message. Ignoring it : %s" % msg)
                    continue

                if msg["type"] == "stop":
                    self.stopExecutor(msg)
                    break
                elif msg["type"] == "start":
                    self.startExecutor(msg)
                elif msg["type"] == "startAction":
                    self.startAction(msg)
                elif msg["type"] == "stopAction":
                    self.stopAction(msg)
                else:
                    logger.warning("Executor received unknown message %s" % msg)
                    
            self.update()
            self.stopEvent.wait(0.1)
            
        logger.info("Executor stopped")

    def __del__(self):
        del self.actionExecutor