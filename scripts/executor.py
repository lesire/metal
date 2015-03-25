from __future__ import division

try:
    import Queue
    version = 2
except:
    import queue as Queue
    version = 3

import logging; logger = logging.getLogger("hidden")

import threading
import time

class Executor(threading.Thread):
    def __init__ (self, inQueue, outQueue, actionExecutor=None):
        self.inQueue = inQueue
        self.outQueue = outQueue
        
        self.actionExecutor = actionExecutor

        self.isStarted = False
        self.nextEvents = [] #list of tuple (time, action)
        self.beginDate = -1
        threading.Thread.__init__ (self, name="Executor")

    # get readable time
    def user_time(self, t):
        return int(round(1000*(t-self.beginDate)))

    # end action callback
    def action_callback(self, action):
        currentTime = time.time()
        self.nextEvents.append((currentTime, action))
        logger.info("Callback of action {a} at time {t}".format(a=action["name"],t=self.user_time(currentTime)/1000))

    #called periodically
    def update(self):
        currentTime = time.time()
        
        self.actionExecutor.update()
        
        for action in [a for t,a in self.nextEvents if t <= currentTime]:
            self.outQueue.put({"type":"endAction", "action":action, "time": self.user_time(currentTime)})
            logger.info("End of action %s" % action["name"])
            logger.debug("End of action %s" % action)
            
        self.nextEvents = [(t,a) for t,a in self.nextEvents if t > currentTime]
        
    def startAction(self, msg):
        if "action" not in msg or type(msg["action"]) != dict:
            logger.error("Executor received an ill formated action : %s" % msg)
            return
        
        action = msg["action"]
        currentTime = time.time()
        logger.info("Start of action {a} at time {t}".format(a=action["name"],t=self.user_time(currentTime)/1000))

        try:
            self.actionExecutor.execute(action, self.action_callback)
        except AttributeError:
            logger.error("Cannot execute %s." % action["name"])
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
        pass
    
    def run(self):
        logger.info("Executor launched")
        
        while True:
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
            time.sleep(0.1)
            
        logger.info("Executor stopped")