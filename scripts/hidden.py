#!/usr/bin/env python3

import argparse
import os
import sys
import threading
import time
import signal

try:
    import Queue
    version = 2
except:
    import queue as Queue
    version = 3

import logging; logger = logging.getLogger("hidden")

import executor
import supervisor
from executors.action_executor import *

"""
Messages are dictionnary, with at least the key "type" that can be :
 - start        (supervisor -> executor): beginning of the execution
 - stop         (supervisor -> executor)
 - startAction  (supervisor -> executor) : for uncontrollable and controllable actions
 - endAction    (executor -> supervisor) : for uncontrollable and controllable actions
 - stopAction    (supervisor -> executor) : for controllable actions
 - alea         (executor -> supervisor)

The field "time" is an int, in ms, measured from the startTime given in the "start" message
"""

"""
Filter used to filter the logs based on their filenames
"""
class FileFilter(logging.Filter):
    def __init__(self, l):
        self.l = l

    def filter(self, record):
        return os.path.splitext(os.path.basename(record.pathname))[0] in self.l

class Hidden:
    def __init__(self):
        self.threadSupervisor = None
        #self.logger = logging.getLogger('hidden')

    def init(self):
        parser = argparse.ArgumentParser(description='Execute a plan')
        parser.add_argument('--logLevel', type=str, default="info")
        parser.add_argument('--logFilter', type=str, nargs="*", choices=["hidden", "supervisor", "executor", "action_executor"], metavar="filename")
        parser.add_argument('--agentName', metavar="agent", type=str)
        parser.add_argument('--executor', type=str, choices=["morse", "dummy", "dummy-ma", "delay", "delay-ma"], default="dummy", metavar="action executor (eg., 'morse')")
        parser.add_argument('--waitSignal', action="store_true")
        parser.add_argument('planFile', metavar='plan', type=str)
        args = parser.parse_args()

        #Configure the logger
        numeric_level = getattr(logging, args.logLevel.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % args.logLevel)
        sh = logging.StreamHandler()
        sh.setLevel(numeric_level)
        sh.setFormatter(logging.Formatter('%(levelname)s(%(filename)s:%(lineno)d):%(message)s'))
        logger.addHandler(sh)

        if args.logFilter is not None:
            f = FileFilter(args.logFilter)
            logger.addFilter(f)
    
        #Get the plan
        if not os.access(args.planFile, os.R_OK):
            logger.error("Cannot open plan file : %s" % args.planFile)
            sys.exit(1)
        
        with open(args.planFile) as f:
            planString = " ".join(f.readlines())
        logger.info("Plan read from file %s" % args.planFile)

        ex = self.createExecutor(args.executor, args.agentName)
        logger.info("Threads created")

        self.launchAgentArchitecture(ex, planString, args.agentName)
        logger.info("Execution thread launched")
    
        self.started = False
        if args.waitSignal:
            self.waitSignal()
        else:
            self.startCallback('auto', None)

    def startCallback(self, signum, frame=None):
        logger.info('Start callback called with signal %s' % signum)
        if self.started:
            logger.info("Supervisor already started: doing nothing")
        elif self.threadSupervisor is not None:
            logger.info("Starting Supervisor")
            self.threadSupervisor.start()
            self.started = True
        else:
            logger.error("Supervisor thread not initialized!")

    def waitSignal(self):
        signal.signal(signal.SIGUSR1, self.startCallback)
        logger.info("Waiting for signal USR1 to start. Example : kill -USR1 %s" % os.getpid())
      
    def createExecutor(self, name, agentName):
        if name == "morse":
            from executors.morse_executor import MORSEActionExecutor
            return MORSEActionExecutor()
        elif name == "dummy":
            return DummyActionExecutor(agentName=agentName)
        elif name == "dummy-ma" or name == "delay-ma":
            if agentName is None:
                logger.error("Cannot use executor dummy-ma without an agent name")
                sys.exit(1)
            folder = "/tmp/hidden2"
            if not os.path.exists(folder):
                os.makedirs(folder)
            for f in os.listdir(folder):
                os.remove(os.path.join(folder, f))
            if name == "dummy-ma":
                return DummyMAActionExecutor(agentName, folder)
            else:
                return DummyDelayMA(agentName, folder)
        elif name == "delay":
            return DummyDelay(agentName=agentName)
        else:
            return None

    def launchAgentArchitecture(self, ex, planString, agentName):
        self.q1 = Queue.Queue() 
        self.q2 = Queue.Queue() 
    
        self.threadSupervisor = supervisor.Supervisor(self.q1, self.q2, planString, agent=agentName)
        self.threadExecutor = executor.Executor(self.q2, self.q1, ex)
    
        #threadSupervisor.start()
        self.threadExecutor.start()
    
    def main(self):
        while threading.active_count() > 1:
            time.sleep(1)
    
if __name__=="__main__":
    try:
        h = Hidden()
        h.init()
        h.main()
    except KeyboardInterrupt:
        os._exit(1)
