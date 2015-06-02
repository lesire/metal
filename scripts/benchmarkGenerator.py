#!/usr/bin/env python3

import argparse
import json
import os
import random
import re
import sys

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
    print("Found %d agents : %s" % (len(agents), list(agents)))
    
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
            
    print("Nominal plan last for %.2f seconds" % result)
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
    parser.add_argument("--mission"           , type=os.path.abspath, required=True)
    args = parser.parse_args(argv)

    os.makedirs(args.outputFolder)
    
    planFile, planPDDLFile = getPlanFiles(args.mission)
    agents = getAgentList(planFile)
    planLength = getPlanLength(planPDDLFile)
    
    generateDead(args.outputFolder, agents=agents, planLength=planLength)
    

if __name__=="__main__":
    main(sys.argv[1:])