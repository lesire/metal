#!/usr/bin/env python3

import argparse
import os
import sys
import threading
import time
import signal

try:
    import Queue
    #version = 2
except:
    import queue as Queue
    #version = 3

import logging; logger = logging.getLogger("hidden")

import executor
import supervisor
from executors import create_executor, executors

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

    def getDefaultPDDL(self):
        return None

    def init(self, argv):
        parser = argparse.ArgumentParser(description='Execute a plan')
        parser.add_argument('--logLevel', type=str, default="info")
        parser.add_argument('--logFilter', type=str, nargs="*", choices=["hidden", "supervisor", "executor", "action_executor"], metavar="filename")
        parser.add_argument('--agentName', metavar="agent", type=str)
        parser.add_argument('--executor', type=str, choices=executors(), default="dummy", metavar="action executor (eg., 'morse')")
        parser.add_argument('--waitSignal', action="store_true")
        #parser.add_argument('--repairRos', action="store_true")
        parser.add_argument('--planFile', metavar='plan', type=str)
        args = parser.parse_args(argv)

        #Configure the logger
        numeric_level = getattr(logging, args.logLevel.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % args.logLevel)
        sh = logging.StreamHandler()
        logger.setLevel(numeric_level)
        sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s %(process)d (%(filename)s:%(lineno)d): %(message)s'))
        logger.addHandler(sh)

        if args.logFilter is not None:
            f = FileFilter(args.logFilter)
            logger.addFilter(f)
    
        #Get the plan and the associated PDDL files
        if args.planFile is None:
            pddlFiles = self.getDefaultPDDL()
            if pddlFiles is None or "plan" not in pddlFiles:
                logger.error("No plan to execute : nothing read from the command line or any other means")
                sys.exit(1)
        elif not os.access(args.planFile, os.R_OK):
            logger.error("Cannot open plan file : %s" % args.planFile)
            sys.exit(1)
        else:
            with open(args.planFile) as f:
                planString = " ".join(f.readlines())
            logger.info("Plan read from file %s" % args.planFile)
            pddlFiles = {"plan" : planString}

            d = os.path.dirname(os.path.abspath(args.planFile))
            prefix = os.path.basename(args.planFile).split(".")[0]
            domain = os.path.join(d, prefix + "-domain.pddl")
            prb = os.path.join(d, prefix + "-prb.pddl")
            prbHelper = os.path.join(d, prefix + "-prb-helper.pddl")
            if not os.access(domain, os.R_OK) or \
               not os.access(prb, os.R_OK) or \
               not os.access(prbHelper, os.R_OK):
                logger.warning("Cannot find the pddl file. Will not be able to repair.")
                logger.warning("Looking for %s %s %s" % (domain, prb, prbHelper))
            else:
                with open(domain) as f:
                    pddlFiles["domain"] = f.read()
                with open(prb) as f:
                    pddlFiles["prb"] = f.read()
                with open(prbHelper) as f:
                    pddlFiles["helper"] = f.read()
        
        ex = create_executor(args.executor, agentName=args.agentName, folder='/tmp/hidden2')
        #self.createExecutor(args.executor, args.agentName)
        logger.info("Threads created")

        self.launchAgentArchitecture(ex, pddlFiles["plan"], args.agentName, pddlFiles=pddlFiles)
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

    def createSupervisor(self, plan, agent, pddlFiles):
        return supervisor.Supervisor(self.q1, self.q2, plan, agent=agent, pddlFiles=pddlFiles)

    def launchAgentArchitecture(self, ex, planString, agentName, pddlFiles=None, repairRos = False):
        self.q1 = Queue.Queue() 
        self.q2 = Queue.Queue() 
    
        self.threadSupervisor = self.createSupervisor(planString, agentName, pddlFiles)
        self.threadExecutor = executor.Executor(self.q2, self.q1, ex, agent=agentName)
    
        #threadSupervisor.start()
        self.threadExecutor.start()
    
    def main(self):
        while threading.active_count() > 1:
            time.sleep(1)
    
if __name__=="__main__":
    try:
        h = Hidden()
        h.init(sys.argv[1:])
        h.main()
    except KeyboardInterrupt:
        os._exit(1)
