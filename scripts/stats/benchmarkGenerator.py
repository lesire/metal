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

sh = logging.StreamHandler()
sh.setFormatter(logging.Formatter('%(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
logger.addHandler(sh)

# return a sorted list of n random values between minValue and maxValue with a minimum distance of sep between each of them
def randomList(n, minValue, maxValue, sep):
    result = []
    for i in range(n):
        newValue = random.uniform(minValue, maxValue)
        while result and min([abs(newValue - r) for r in result]) < sep:
            newValue = random.uniform(minValue, maxValue)
        result.append(newValue)
    return sorted(result)

class AbstractPlanGen(object):
    def __init__(self, mission, planLength, nbrInstance, **kwargs):
        self.robots = list([str(k) for k in mission["agents"].keys()])
        self.planLength = planLength
        self.nbrInstance = nbrInstance
        self.mission = mission

        self.activeRobots = [str(k) for k,a in mission["agents"].items() if not a["spare"]]

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

    def addDelayedAction(self, delayedRobot, time, delay):
        
        self.data[self.nextIndex()] = {"type" : "delay", "date":time, "to":delayedRobot, "data":{"delay":float(delay)}}

    #Must define a _nextAlea method that acts on self.data

class NominalGen(AbstractPlanGen):
    _name = "nominal"
    _description = "No alea."
    
    def _nextAlea(self):
        pass
        
class DeadRobotGen(AbstractPlanGen):
    _name = "deadRobot"
    _description = "A random robot is declared dead, another robot is notified after (might be quite late)."
    
    def _nextAlea(self):
        deadRobot,reparingRobot = random.sample(self.activeRobots, 2)
        d1 = random.uniform(0, self.planLength)  # death of the robot
        d2 = random.uniform(d1, self.planLength) # notification to a live robot
        
        self.addDeadRobot(deadRobot, d1, reparingRobot, d2)


class deadRobotIsolatedRobotGen(AbstractPlanGen):
    _name = "deadRobotIsolatedRobot"
    _description = "A random robot is declared dead, another robot is notified after (might be quite late). During this time, a third robot is isolated."
    
    def _nextAlea(self):
        deadRobot,reparingRobot,isolatedRobot = random.sample(self.activeRobots, 3)
        d1 = random.uniform(5, self.planLength-5)  # death of the robot
        d2 = random.uniform(d1, self.planLength-5) # notification to a live robot
        d3 = random.uniform(d2, self.planLength) # end of the isolation
       
        self.addDeadRobot(deadRobot, d1, reparingRobot, d2)
        self.addIsolatedRobot(isolatedRobot, d1-5, d3)
    
class TargetFoundGen(AbstractPlanGen):
    _name = "targetFound"
    _description = "A random robot finds a target"
    
    def _nextAlea(self):
        detectorRobot,repairingRobot = random.sample(self.activeRobots, 2)
        d1 = random.uniform(0, self.planLength-10)  # target found
        
        self.addTargetFound(detectorRobot, d1, repairingRobot, d1+5)


class TargetFoundIsolatedRobotGen(AbstractPlanGen):
    _name = "targetFoundIsolatedRobot"
    _description = "A random robot finds a target while another one is isolated"
    
    def _nextAlea(self):
        detectorRobot,repairingRobot,isolatedRobot = random.sample(self.activeRobots, 3)
        d1 = random.uniform(5, self.planLength-5)  # target found
        d2 = random.uniform(d1, self.planLength-5) # Coms back online
        
        self.addTargetFound(detectorRobot, d1, repairingRobot, d1+5)
        self.addIsolatedRobot(isolatedRobot, d1-5, d2)

class DoubleTargetFoundGen(AbstractPlanGen):
    _name = "doubleTargetFound"
    _description = "Two random different robots find a target !"

    def _nextAlea(self):
        detectorRobot,repairingRobot,detectorRobot2,repairingRobot2 = random.sample(self.activeRobots, 4)
        d1,d2 = randomList(2, 0, self.planLength-10, 25)  # targets found

        self.addTargetFound(detectorRobot, d1, repairingRobot, d1+5)
        self.addTargetFound(detectorRobot2, d2, repairingRobot2, d2+5)

class simpleDelayGen(AbstractPlanGen):
    _name = "simpleDelay"
    _description = "A random robot has a delay of 45 sec"
    
    def _nextAlea(self):
        delayedRobot = random.sample(self.activeRobots, 1)
        d1 = random.uniform(5, self.planLength-5) # Delayed action
        
        self.addDelayedAction(delayedRobot, d1, 45)

class simpleDelayIsolatedRobotGen(AbstractPlanGen):
    _name = "simpleDelayIsolatedRobot"
    _description = "A random robot has a delay of 45 sec  while another one is isolated"
    
    def _nextAlea(self):
        delayedRobot,isolatedRobot = random.sample(self.activeRobots, 2)
        d1 = random.uniform(5, self.planLength-5)  # Delayed action
        d2 = random.uniform(d1, self.planLength-5) # Coms back online
        
        self.addDelayedAction(delayedRobot, d1, 45)
        self.addIsolatedRobot(isolatedRobot, d1-5, d2)

class complexGen(AbstractPlanGen):
    _name = "complex"
    _description = "2 random aleas are created"
    
    def _nextAlea(self):
        dates = sorted([random.uniform(5, self.planLength-5) for _ in range(2)])
        while dates[1] - dates[0] < 20:
            dates = sorted([random.uniform(5, self.planLength-5) for _ in range(2)])        

        self.availRobots = set(self.activeRobots)

        for d in dates:
            type = random.choice(["dead", "deadIsolated", "target", "targetIsolated", "delay"])
            if type == "dead":
                deadRobot,reparingRobot = random.sample(list(self.availRobots), 2)
                self.addDeadRobot(deadRobot, d-2, reparingRobot, d+2)
                self.availRobots.remove(deadRobot)
            elif type == "deadIsolated":
                deadRobot,reparingRobot,isolatedRobot = random.sample(list(self.availRobots), 3)
                self.addDeadRobot(deadRobot, d-2, reparingRobot, d+2)
                self.addIsolatedRobot(isolatedRobot, d-5, d+5)
                self.availRobots.remove(deadRobot)
            elif type == "target":
                detectorRobot,repairingRobot = random.sample(list(self.availRobots), 2)
                self.addTargetFound(detectorRobot, d-2, repairingRobot, d+2)
                self.availRobots.remove(detectorRobot)
            elif type == "targetIsolated":
                detectorRobot,repairingRobot,isolatedRobot = random.sample(list(self.availRobots), 3)
                self.addTargetFound(detectorRobot, d-2, repairingRobot, d+2)
                self.addIsolatedRobot(isolatedRobot, d-5, d+5)
                self.availRobots.remove(detectorRobot)
            elif type == "delay":
                delayedRobot = random.sample(list(self.availRobots), 1)
                self.addDelayedAction(delayedRobot, d, 45)
            else:
                logger.error("Unknown type")


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

def getMission(missionFolder):
    if missionFolder.endswith("/"):
        missionFolder = missionFolder[:-1]
    missionFile = missionFolder + ".json"
    with open(missionFile) as f:
        mission = json.load(f)
        
    for agent in mission["agents"].keys():
        if "spare" in mission["agents"][agent]:
            if mission["agents"][agent]["spare"] in ["false", "False"]:
                mission["agents"][agent]["spare"] = False
            elif mission["agents"][agent]["spare"] in ["true", "True"]:
                mission["agents"][agent]["spare"] = True
            else:
                logging.error("Cannot convert %s to boolean" % mission["agents"][agent]["spare"])
                sys.exit(1)
        else:
            mission["agents"][agent]["spare"] = False

        for k,v in mission["agents"][agent]["position"].items():
            mission["agents"][agent]["position"][k] = float(v)
            
    for wpg in mission["wp_groups"].keys():
        for wpn in mission["wp_groups"][wpg]["waypoints"].keys():
            for k,v in mission["wp_groups"][wpg]["waypoints"][wpn].items():
                mission["wp_groups"][wpg]["waypoints"][wpn][k] = float(v)

    for obs in mission["mission_goal"]["observation_points"].keys():
        for k,v in mission["mission_goal"]["observation_points"][obs].items():
            mission["mission_goal"]["observation_points"][obs][k] = float(v)
                
    return mission
    
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

def main(argv):
    import simu_stats
    
    parser = argparse.ArgumentParser(description="Launch a statistical simulation")
    parser.add_argument("--outputFolder"   , type=os.path.abspath, required=True)
    parser.add_argument("--mission"        , type=os.path.abspath, required=True)
    parser.add_argument("-n", "--nbrInstance", type=int, default=10)
    parser.add_argument("-j", "--jobs"     , type=int, default=20)
    parser.add_argument("--logLevel"       , type=str, default="info")
    parser.add_argument("--dry-run"        , action="store_true")
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
    mission = getMission(args.mission)
    planLength = getPlanLength(planPDDLFile)
    
    nbrInstance = args.nbrInstance
    
    logger.info("Creating the alea files")
    
    for gen in AbstractPlanGen.__subclasses__():
    #for gen in [NominalGen]:
        #if gen == NominalGen: continue #do not run nominal
        logger.info("Creating problems with generator %s" % gen._name)
        
        os.chdir(args.outputFolder)

        #make directory
        dirBench = "aleas_" + gen._name
        os.mkdir(dirBench) 

        g = gen(mission=mission, planLength=planLength, nbrInstance=nbrInstance)
        for i in range(nbrInstance):
            d = g.nextAlea()
            with open(os.path.join(dirBench, str(i) + ".json"), "w") as f:
                json.dump(d, f, sort_keys=True, indent=2)
    
        logger.info("Finished creating the aleas files for %s. Now running the benchmarks" % gen._name)

        if not args.dry_run:
            simu_stats.runBenchmark(args.mission, sorted([os.path.join(dirBench, f) for f in os.listdir(dirBench)]), os.path.abspath("output_" + gen._name), maxJobs=args.jobs, vnet=True)
        
        logger.info("Finished running the benchmarks for %s." % gen._name)
        
    logger.info("Done")
            
if __name__=="__main__":
    main(sys.argv[1:])
