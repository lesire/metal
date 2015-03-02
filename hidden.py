#!/usr/bin/env python3

import argparse
import logging
import os
import sys
import threading
import time

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
 - startAction  (supervisor -> executor)
 - endAction    (executor -> supervisor)
 - alea         (executor -> supervisor)

The field "time" is an int, in ms, measured from the startTime given in the "start" message
"""

def main(argv):
    parser = argparse.ArgumentParser(description='Execute a plan')
    parser.add_argument('planFile', metavar='plan', type=str)
    parser.add_argument('--logLevel', type=str, default="info")
    parser.add_argument('--agentName', metavar="agent", type=str)
    parser.add_argument('--executor', type=str, metavar="action executor (eg., 'morse')")
    args = parser.parse_args()
    
    #Configure the logger
    numeric_level = getattr(logging, args.logLevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.logLevel)
    logging.basicConfig(level=numeric_level, format='%(levelname)s(%(filename)s:%(lineno)d):%(message)s')
    
    #Get the plan
    if not os.access(args.planFile, os.R_OK):
        logging.error("Cannot open plan file : %s" % args.planFile)
        sys.exit(1)
    
    q1 = Queue.Queue() 
    q2 = Queue.Queue() 
    
    with open(args.planFile) as f:
        planString = " ".join(f.readlines())

    if 'morse' in args.executor:
        ex = MORSEActionExecutor()
    else:
        ex = None

    threadSupervisor = supervisor.Supervisor(q1, q2, planString, agent=args.agentName)
    threadExecutor = executor.Executor(q2, q1, ex)
    
    threadSupervisor.start()
    threadExecutor.start()
    
    while threading.active_count() > 1:
        time.sleep(1)
    
if __name__=="__main__":
    import sys
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        import os
        os._exit(1)
