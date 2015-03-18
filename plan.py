from __future__ import division

from copy import copy,deepcopy
from pprint import pprint
import itertools
import json
import logging
import sys

try:
    import pystn
except ImportError:
    print("[error] cannot import pystn")
    sys.exit(1)

def isActionControllable(actionName):
    return "communicate" in actionName

agentList = ["ressac", "ressac1", "ressac2", "mana", "minnie", "momo"]

timeDelta = 1
timeFactor = 1000 # all float duration are multiplied by this and then cast into int

class PlanImportError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


class Plan:
    def __init__(self, planStr, agent=None):
        d = deepcopy(json.loads(planStr))
        self.jsonDescr = d

        self.agent = agent
        if agent is None:
            self.stn = pystn.STNIntMa(pystn.StnType.GraphSTN)
        else:
            self.stn = pystn.STNIntMa(pystn.StnType.GraphSTN, agent)
        
        self.tpName = {}
        self.tpName[1] = "1-end"
        self.stn.addPoint("1-end", self.stn.getAgentName())
        
        self.absTimes = []
        
        #Dictionary. Key is the previous action index. Value is the dictionnary of new index
        splittedAction = {}
        splittedTps = {} #Keys are tp
        
        self.actions = copy(d["actions"])
        
        maxTp = max([max(a["startTp"], a["endTp"]) for a in self.actions.values()])
        
        #first split coordinating action:
        for index,a in d["actions"].items():
            if "communicate" in a["name"]:
                l = a["name"].split(" ")
                if l[1] not in agentList or l[2] not in agentList:
                    logging.error("cannot know if %s is a coordination action that should be split in 2" % a["name"])
                    continue
                    
                if len(l) > 3 and l[3] in agentList:
                    logging.error("Cannot know if %s is a coordination action that should be split in 2. Found 3 agents ?!?")
                    continue
                
                splittedAction[index] = {}
                splittedTps[a["startTp"]] = {}
                splittedTps[a["endTp"]] = {}
                
                for agent in l[1],l[2]:
                    newA = deepcopy(a)
                    
                    newA["startTp"] = maxTp + 1
                    newA["endTp"] = maxTp + 2
                    newA["agent"] = agent
                    splittedTps[a["startTp"]][agent] = maxTp + 1
                    splittedTps[a["endTp"]][agent] = maxTp + 2
                    maxTp += 2
                    
                    newIndex = index + "-" + agent
                    splittedAction[index][agent] = newIndex
                    self.actions[newIndex] = newA
                del self.actions[index]
            elif self.agent is not None:
                if a["name"] == "dummy init" or a["name"] == "dummy end":
                    a["agent"] = agent
                else:
                    
                    if a["name"].startswith("dummy "):
                        n = a["name"].split(" ")[3]
                    else:
                        n = a["name"].split(" ")[1]
                    if not n in agentList:
                        logging.error("Unknown agent %s for action %s" % (n, a["name"]))
                        continue
                    a["agent"] = n

        if self.agent is not None:
            for a in set(a["agent"] for a in self.actions.values()):
                if a != self.agent:
                    self.stn.addAgent(a)

        for index,a in self.actions.items():
            tpNumber = a["startTp"]
            if tpNumber not in self.tpName:
                self.tpName[tpNumber] = str(tpNumber) + "-start-" + a["name"]
            a["tStart"] = self.tpName[tpNumber]
            
            tpNumber = a["endTp"]
            if tpNumber not in self.tpName:
                self.tpName[tpNumber] = str(tpNumber) + "-end-" + a["name"]
            a["tEnd"] = self.tpName[tpNumber]
                
            if isActionControllable(a["name"]):
                a["controllable"] = True
            else:
                a["controllable"] = False
            
            if "children" in a and a["children"]:
                a["abstract"] = True
            #else:
            #    a["abstract"] = False
                
            if not "executed" in a:
                a["executed"] = False
            if not "locked" in a:
                a["locked"] = False
            a["dMin"] = abs(a["dMin"])
            
            self.actions[index] = a

        for a in self.actions.values():

            if a["tStart"][0] != "0" and a["tStart"] not in self.stn.getNodeIds():
                if self.agent is not None:
                    self.stn.addPoint(a["tStart"], a["agent"])
                else:
                    self.stn.addPoint(a["tStart"])
                self.stn.addConstraint(a["tStart"], self.tpName[1], 0)
            if a["tEnd"][0] != "0" and a["tEnd"] not in self.stn.getNodeIds():
                if self.agent is not None:
                    self.stn.addPoint(a["tEnd"], a["agent"])
                else:
                    self.stn.addPoint(a["tEnd"])
                self.stn.addConstraint(a["tEnd"], self.tpName[1], 0)
            
            if a["tStart"] != a["tEnd"]:
                if "dMin" in a:
                    self.stn.addConstraint(a["tStart"], a["tEnd"], int(round(timeFactor*a["dMin"])))

                else:
                    logging.warning("Action %s does not have a dMin ?" % a["name"])
                #if any([re.match(regex, a["name"]) for regex in nonRandomAction]):
                #     self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(timeFactor*a["dMin"]), int(timeFactor*a["dMin"]))

        # 1 is the end point, should be after every one else
        for node in self.stn.getNodeIds():
            if node != self.tpName[1]:
                self.stn.addConstraint(node, self.tpName[1], timeDelta)

        for cl in (d["causal-links"] + d["temporal-links"]):
            starts = []
            ends = []
            
            for tpKey,tpList in zip(["startTp","endTp"],[starts,ends]):
                if cl[tpKey] in splittedTps:
                    # causal link from a split action
                    for agent in splittedTps[cl[tpKey]]:
                        newTp = self.tpName[splittedTps[cl[tpKey]][agent]]
                        tpList.append(newTp)
                else:
                    newTp = self.tpName[cl[tpKey]]
                    tpList.append(newTp)

            for start,end in itertools.product(starts, ends):
                if start.startswith("0-"):
                    self.stn.addConstraint(self.stn.getStartId(), end, timeDelta)
                else:
                    self.stn.addConstraint(start, end, timeDelta)

        for action in d["actions"].values():
            if "children" in action and action["children"]:
                for child in action["children"][1:-1]:
                    self.stn.addConstraint(action["tStart"], self.actions[child]["tStart"], 2*timeDelta)
                    self.stn.addConstraint(self.actions[child]["tEnd"], action["tEnd"], 2*timeDelta)
                
                    logging.debug("Adding %s timedelta before %s (hierarchy)" % (action["tStart"], self.actions[child]["tStart"]))
                    logging.debug("Adding %s timedelta before %s (hierarchy)" % (self.actions[child]["tEnd"], action["tEnd"]))
                
                    if not self.stn.isConsistent():
                        logging.error("**  Error : invalid STN when importing the abstract links")
                        raise PlanImportError("invalid STN when importing the abstract links")

        if "absolute-time" in d:
            for time, value in d["absolute-time"]:
                
                value = int(round(value*timeFactor))
                
                if time in splittedTps:
                    tps = splittedTps[time].values()
                else:
                    tps = [time]

                for t in tps:
                    logging.debug("Adding %s at exactly %s" % (self.tpName[t], value))
                    logging.debug("Bounds are %s" % self.stn.getBounds(self.tpName[t]))
                    if not self.stn.mayBeConsistent(self.stn.getStartId(), self.tpName[t], value, value):
                        logging.error("**  Error : invalid STN when importing the plan and setting %s at %s" % (self.tpName[t], value))
                        raise PlanImportError("invalid STN when importing the plan and setting %s at %s" % (self.tpName[t], value))

                    self.stn.addConstraint(self.stn.getStartId(), self.tpName[t], value, value)
                    self.absTimes.append( (self.tpName[t], value) )

        if "unavailable-actions" in d:
            for forbiddenAction in d["unavailable-actions"]:
                if [a["name"] for a in self.actions if a["name"] == forbiddenAction and not a["executed"]]:
                    logging.error("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)
                    raise PlanImportError("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)

        if not self.stn.isConsistent():
            logging.error("**  Error : invalid STN when importing the plan")
            raise PlanImportError("invalid STN when importing the plan")
        
        
        #remove dummy init/end actions
        indexToRemove = []
        for index in self.actions.keys():
            a = self.actions[index]
            if a["name"].startswith("dummy") and a["name"] != "dummy init" and a["name"] != "dummy end":
                indexToRemove.append(index)
        
        self.actions = {key: self.actions[key] for key in self.actions if key not in indexToRemove}
        
        tps = []
        for tp in self.stn.getNodeIds():
            if tp != "1-end":
                tps.append((tp, self.stn.getBounds(tp).lb))
        
        tps.sort(key = lambda x:x[1])
        
        logging.debug("Nominal plan")
        for tp,time in tps:
            tpName = tp.split("-")[0]
            position = tp.split("-")[1]
            actionName = "-".join(tp.split("-")[2:])
            
            if position == "start":
                logging.debug("\t%5.2f (%s): %s" % (time/1000, tpName, actionName))

    def getLength(self):
        return self.stn.getLength()/timeFactor

    def setDeadline(self, deadline):
        self.stn.setHorizon(int(round(deadline*timeFactor)))
        
        if not self.stn.isConsistent():
            logging.warning("STN not valid after the set up of the deadline")
            return False
        return True
    
    def getJsonDescription(self):
        logging.error("Export in Json not implemented yet if some actions are splitted")
        raise Exception("Not Implemented yet")
        return self.jsonDescr
    
    #assume value in ms
    def setTimePoint(self, tpName, value):
        if not "absolute-time" in self.jsonDescr:
            self.jsonDescr["absolute-time"] = []
            
        #add it in the plan description
        valueSec = float(value)/timeFactor
        self.jsonDescr["absolute-time"].append([int(tpName.split("-")[0]), valueSec])

        if not self.stn.mayBeConsistent(self.stn.getStartId(), tpName, value, value):
            logging.warning("Calling set timepoint for %s at %s" % (tpName, value))
            logging.warning("STN will not be consistent. Bonds are : %s" % self.stn.getBounds(tpName))

        #add this constraint in the STN
        self.stn.addConstraint(self.stn.getStartId(), tpName, value, value)

        for index,action in self.jsonDescr["actions"].items():
            
            # The action starts with this tp
            if  int(tpName.split("-")[0]) == action["startTp"] and "dummy" not in action["name"]:
                action["executed"] = True
                action["locked"] = True
                
            if  int(tpName.split("-")[0]) == action["endTp"] and "dummy" not in action["name"]:

                #The action ends with this tp : update the action lengths
                if "dMax" in action:
                    start = self.stn.getConstraint(self.stn.getStartId(), str(action["tStart"]))
                    if start.ub != start.lb:
                        logging.error(tpName)
                        logging.error("Error : an action is executed but still has temporal flexibility %s,%s?" % (start.lb, start.ub))
                        sys.exit(1)
                        return
                
                    duration = (value - start.lb)/timeFactor

                    action["dMax"] = max(duration, action["dMax"])
                
            self.jsonDescr["actions"][index] = action

        logging.debug("Executing %s at %s" % (tpName, value))

    def setActionUnavailable(self, action):
        if not "unavailable-actions" in self.jsonDescr:
            self.jsonDescr["unavailable-actions"] = []
            
        self.jsonDescr["unavailable-actions"].append(action["name"])
