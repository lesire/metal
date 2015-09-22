#!/usr/bin/env python
### rosbag API is not python3 compatible ###

from __future__ import division

import argparse
import json
import logging; logger = logging.getLogger(__name__)
import os
import random
import re
import sys

import simu_stats

sh = logging.StreamHandler()
sh.setFormatter(logging.Formatter('%(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
logger.addHandler(sh)

class AbstractPlanGen(object):
    def __init__(self, robots, planLength, nbrInstance, **kwargs):
        self.robots = list(robots)
        self.planLength = planLength
        self.nbrInstance = nbrInstance
        
    #Store the output in self.data.
    def nextAlea(self):
        self.data = {}
        
        self._nextAlea()
        
        return self.data
    
    def nextIndex(self):
        for i in range(1000):
            if not str(i) in self.data:
                return str(i)
    
    def addDeadRobot(self, deadRobot, deadTime, reparingRobot, repairingTime):
        self.data[self.nextIndex()] = {"type" : "robotDead", "date":deadTime, "to":deadRobot, "data":{"robot" : deadRobot}}
        self.data[self.nextIndex()] = {"type" : "robotDead", "date":repairingTime, "to":reparingRobot, "data":{"robot" : deadRobot}}
        
    def addIsolatedRobot(self, isolatedRobot, startTime, endTime):
        self.data[self.nextIndex()] = {"type" : "add", "date":startTime, "to":"vnet", "data":{"src":isolatedRobot, "tgt":"*", "filter":"block", "bidir":True}}
        self.data[self.nextIndex()] = {"type" : "del", "date":endTime, "to":"vnet", "data":{"src":"*", "tgt":"*", "index":"*"}}

    def addTargetFound(self, detectorRobot, detectionTime, repairingRobot, repairingTime, pos=(200,100), trackingRobot = [], endTrackingTime = None):
        
        if endTrackingTime is None:
            endTrackingTime = detectionTime + 60 # 1 min
        self.data[self.nextIndex()] = {"type" : "targetFound", "date":detectionTime, "to":detectorRobot, "data":{"target":{"x" : pos[0], "y" : pos[1]}}}
        
        for r in trackingRobot:
            if r != detectorRobot:
                self.data[self.nextIndex()] = {"type" : "state", "date":detectionTime+1, "to":r, "data":{"state":"tracking"}}

        self.data[self.nextIndex()] = {"type" : "state", "date":repairingTime, "to":repairingRobot, "data":{"state":"repairingactive"}}

        # Must finish the track to finish the plan
        self.data[self.nextIndex()] = {"type" : "state", "date":endTrackingTime, "to":detectorRobot, "data":{"state":"running"}}

    #Must define a _nextAlea method that acts on self.data

class DeadRobotGen(AbstractPlanGen):
    _name = "deadRobot"
    _description = "A random robot is declared dead, another robot is notified after (might be quite late)."
    
    def _nextAlea(self):
        deadRobot,reparingRobot = random.sample(self.robots, 2)
        d1 = random.uniform(0, self.planLength)  # death of the robot
        d2 = random.uniform(d1, self.planLength) # notification to a live robot
        
        self.addDeadRobot(deadRobot, d1, reparingRobot, d2)


class deadRobotIsolatedRobotGen(AbstractPlanGen):
    _name = "deadRobotIsolatedRobot"
    _description = "A random robot is declared dead, another robot is notified after (might be quite late). During this time, a third robot is isolated."
    
    def _nextAlea(self):
        deadRobot,reparingRobot,isolatedRobot = random.sample(self.robots, 3)
        d1 = random.uniform(5, self.planLength-5)  # death of the robot
        d2 = random.uniform(d1, self.planLength-5) # notification to a live robot
        d3 = random.uniform(d2, self.planLength) # end of the isolation
       
        self.addDeadRobot(deadRobot, d1, reparingRobot, d2)
        self.addIsolatedRobot(isolatedRobot, d1-5, d3)
    
class TargetFoundGen(AbstractPlanGen):
    _name = "targetFound"
    _description = "A random robot finds a target"
    
    def _nextAlea(self):
        detectorRobot,repairingRobot = random.sample(self.robots, 2)
        d1 = random.uniform(0, self.planLength-10)  # target found
        
        self.addTargetFound(detectorRobot, d1, repairingRobot, d1+5)


class TargetFoundIsolatedRobotGen(AbstractPlanGen):
    _name = "targetFoundIsolatedRobot"
    _description = "A random robot finds a target while another one is isolated"
    
    def _nextAlea(self):
        detectorRobot,repairingRobot,isolatedRobot = random.sample(self.robots, 3)
        d1 = random.uniform(5, self.planLength-5)  # target found
        d2 = random.uniform(d1, self.planLength-5) # Coms back online
        
        self.addTargetFound(detectorRobot, d1, repairingRobot, d1+5)
        self.addIsolatedRobot(isolatedRobot, d1-5, d2)



def getPlanFiles(missionFolder):
    assert("hipop-files" in os.listdir(missionFolder))
    
    l = os.listdir(os.path.join(missionFolder, "hipop-files"))
    missionName = None
    for f in l:
        if f.endswith("-prb.pddl"):
            missionName = f.replace("-prb.pddl", "")
            break
        
    assert(missionName is not None)
    
    return os.path.join(missionFolder, "hipop-files", missionName + ".plan"),os.path.join(missionFolder, "hipop-files", missionName + ".pddl")

def getAgentList(planFile):
    with open(planFile) as f:
        plan = json.load(f)
        
    agents = set([a["agent"] for a in plan["actions"].values() if "agent" in a])
    logger.info("Found %d agents : %s" % (len(agents), list(agents)))
    
    return agents
    
def getPlanLength(planPDDLFile):
    result = 0
    with open(planPDDLFile) as f:
        for line in f:
            if line.startswith(";"):
                continue
            
            m = re.match("^(\d*(?:.\d*)?)\s*:\s*\((.*)\)\s*\[(\d*(?:.\d*)?)\]", line)
            endAction = float(m.groups()[0]) + float(m.groups()[2])
            result = max(result, endAction)
            
    logger.info("Nominal plan last for %.2f seconds" % result)
    return result
    
def generateDead(outputFolder, agents, planLength):
    
    nbrAlea = 20
    
    for i in range(nbrAlea):
        deadRobot,reparingRobot = random.sample(agents, 2)
        d1 = random.uniform(0, planLength)  # death of the robot
        d2 = random.uniform(d1, planLength) # notification to a live robot
        
        data = {}
        
        data["0"] = {"type" : "robotDead", "date":d1, "to":deadRobot, "data":{"robot" : deadRobot}}
        data["1"] = {"type" : "robotDead", "date":d2, "to":reparingRobot, "data":{"robot" : deadRobot}}
        
        with open(os.path.join(outputFolder, "alea_%d.json"%i), "w") as f:
            json.dump(data, f,indent=2)
        
    pass
    
def main(argv):
    parser = argparse.ArgumentParser(description="Launch a statistical simulation")
    parser.add_argument("--outputFolder"   , type=os.path.abspath, required=True)
    parser.add_argument("--mission"        , type=os.path.abspath, required=True)
    parser.add_argument("-n", "--nbrInstance", type=int, default=10)
    parser.add_argument("-j", "--jobs"     , type=int, default=20)
    parser.add_argument("--logLevel"       , type=str, default="info")
    args = parser.parse_args(argv)
    
    #Configure the logger
    numeric_level = getattr(logging, args.logLevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.logLevel)
    #logger.propagate = False
    logger.setLevel(numeric_level)
    simu_stats.logger=logger #overwrite the logger

    os.makedirs(args.outputFolder)
    
    planFile, planPDDLFile = getPlanFiles(args.mission)
    agents = getAgentList(planFile)
    planLength = getPlanLength(planPDDLFile)
    
    nbrInstance = args.nbrInstance
    
    logger.info("Creating the alea files")
    
    for gen in AbstractPlanGen.__subclasses__():
    #for gen in [TargetFoundGen]:
        logger.info("Creating problems with generator %s" % gen._name)
        
        os.chdir(args.outputFolder)

        #make directory
        dirBench = "aleas_" + gen._name
        os.mkdir(dirBench) 

        g = gen(robots=agents, planLength=planLength, nbrInstance=nbrInstance)
        for i in range(nbrInstance):
            d = g.nextAlea()
            with open(os.path.join(dirBench, str(i) + ".json"), "w") as f:
                json.dump(d, f, sort_keys=True, indent=2)
    
        logger.info("Finished creating the aleas files for %s. Now running the benchmarks" % gen._name)

        simu_stats.runBenchmark(args.mission, sorted([os.path.join(dirBench, f) for f in os.listdir(dirBench)]), os.path.abspath("output_" + gen._name), maxJobs=args.jobs, vnet=True)
        
        logger.info("Finished running the benchmarks for %s." % gen._name)
        
    logger.info("Done")
            
if __name__=="__main__":
    main(sys.argv[1:])