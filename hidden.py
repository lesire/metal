#!/usr/bin/env python3

import argparse
import logging
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

import executor
import supervisor
from action_executor import *

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

supervisorThread = None
def sigHandler(signum, frame):
    logging.info('Signal handler called with signal %s' % signum)
    
    if supervisorThread is not None:
        logging.info("Starting supervision")
        supervisorThread.start()

def launchAgentArchitecture(ex, planString, agentName):
    q1 = Queue.Queue() 
    q2 = Queue.Queue() 
    
    threadSupervisor = supervisor.Supervisor(q1, q2, planString, agent=agentName)
    threadExecutor = executor.Executor(q2, q1, ex)
    

    #threadSupervisor.start()
    threadExecutor.start()
    
    return threadSupervisor
    
def main(argv):
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
    logging.basicConfig(level=numeric_level, format='%(levelname)s(%(filename)s:%(lineno)d):%(message)s')

    if args.logFilter is not None:
        f = FileFilter(args.logFilter)
        logging.getLogger().addFilter(f)
    
    #Get the plan
    if not os.access(args.planFile, os.R_OK):
        logging.error("Cannot open plan file : %s" % args.planFile)
        sys.exit(1)
    
    with open(args.planFile) as f:
        planString = " ".join(f.readlines())

    if args.executor == "morse":
        from executors.morse_executor import MORSEActionExecutor
        ex = MORSEActionExecutor()
    elif args.executor == "dummy":
        ex = DummyActionExecutor(agentName=args.agentName)
    elif args.executor == "dummy-ma" or args.executor == "delay-ma":
        if args.agentName is None:
            logging.error("Cannot use executor dummy-ma without an agent name")
            sys.exit(1)
            
        folder = "/tmp/hidden2"
        if not os.path.exists(folder):
            os.makedirs(folder)
        for f in os.listdir(folder):
            os.remove(os.path.join(folder, f))
            
        if args.executor == "dummy-ma":
            ex = DummyMAActionExecutor(args.agentName, folder)
        else:
            ex = DummyDelayMA(args.agentName, folder)
    elif args.executor == "delay":
        ex = DummyDelay(agentName=args.agentName)
    else:
        ex = None

    s = launchAgentArchitecture(ex, planString, args.agentName)
    
    if args.waitSignal:
        global supervisorThread
        supervisorThread = s
        signal.signal(signal.SIGUSR1, sigHandler)
        logging.info("Waiting for signal USR1 to start. Example : kill -USR1 %s" % os.getpid())
    else:
        s.start()

    while threading.active_count() > 1:
        time.sleep(1)
    
if __name__=="__main__":
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        os._exit(1)
