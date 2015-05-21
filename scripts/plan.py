from __future__ import division

from copy import copy,deepcopy
import itertools
import json
import sys

import logging; logger = logging.getLogger("hidden")

try:
    import pystn
except ImportError:
    print("[error] cannot import pystn")
    sys.exit(1)

def isActionControllable(actionName):
    return "communicate" in actionName

agentList = ["ressac", "ressac1", "ressac2", "mana", "minnie", "momo", "r1", "r2"]

timeDelta = 1
timeFactor = 1000 # all float duration are multiplied by this and then cast into int

class PlanImportError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg

def getAgentFromAction(l):
    if len(l) == 1 or l[1] not in agentList:
        n = l[0].split("_")[1]
    else:
        n = l[1]
        
    if not n in agentList:
        logger.error("Unknown agent %s for action %s" % (n, " ".join(l)))

    return n

class Plan:
    def __init__(self, planStr, agent=None):
        d = json.loads(planStr)
        self.jsonDescr = deepcopy(d)

        self.agent = agent
        if agent is None:
            self.stn = pystn.STNIntMa(pystn.StnType.GraphSTN)
        else:
            self.stn = pystn.STNIntMa(pystn.StnType.GraphSTN, agent)
        
        self.absTimes = []
        
        self.actions = copy(d["actions"])
        
        self.tpName = {}
        self.tpAgent = {}
        
        # End timepoints are duplicated for each agent
        self.tpEnd = {}
        for a in set(a["agent"] for a in self.actions.values() if "agent" in a):
            self.tpEnd[a] = "1-end-" + a
            self.stn.addPoint(self.tpEnd[a], self.stn.getAgentName())
        
        if self.agent is not None:
            for a in set(a["agent"] for a in self.actions.values() if "agent" in a):
                if a != self.agent:
                    self.stn.addAgent(a)

        for index,a in self.actions.items():
            tpNumber = a["startTp"]
            if tpNumber not in self.tpName:
                self.tpName[tpNumber] = str(tpNumber) + "-start-" + a["name"]
                if "agent" in a:
                    self.tpAgent[tpNumber] = a["agent"]
            a["tStart"] = self.tpName[tpNumber]
            
            tpNumber = a["endTp"]
            if tpNumber not in self.tpName:
                self.tpName[tpNumber] = str(tpNumber) + "-end-" + a["name"]
                if "agent" in a:
                    self.tpAgent[tpNumber] = a["agent"]
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
                if self.agent is not None and "agent" in a:
                    self.stn.addPoint(a["tStart"], a["agent"])
                else:
                    self.stn.addPoint(a["tStart"])
                #self.stn.addConstraint(a["tStart"], self.tpName[1], 0)
            if a["tEnd"][0] != "0" and a["tEnd"] not in self.stn.getNodeIds():
                if self.agent is not None and "agent" in a:
                    self.stn.addPoint(a["tEnd"], a["agent"])
                else:
                    self.stn.addPoint(a["tEnd"])
                #self.stn.addConstraint(a["tEnd"], self.tpName[1], 0)
            
            if a["tStart"] != a["tEnd"]:
                if "dMin" in a:
                    self.stn.addConstraint(a["tStart"], a["tEnd"], int(round(timeFactor*a["dMin"])))
                else:
                    logger.warning("Action %s does not have a dMin ?" % a["name"])
                    
                if isActionControllable(a["name"]):
                    self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(timeFactor*a["dMin"]), int(timeFactor*a["dMin"]))

                #if any([re.match(regex, a["name"]) for regex in nonRandomAction]):
                #     self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(timeFactor*a["dMin"]), int(timeFactor*a["dMin"]))

        for cl in (d["causal-links"] + d["temporal-links"]):
            start = self.tpName[cl["startTp"]]

            # If end point is the endTp, then get the one correspondng to the agent executing the action            
            end = self.tpName[cl["endTp"]]
            if cl["endTp"] == 1:
                agent = self.tpAgent[cl["startTp"]]
                end = self.tpEnd[agent]
            
            if start.startswith("0-"):
                self.stn.addConstraint(self.stn.getStartId(), end, timeDelta)
            else:
                self.stn.addConstraint(start, end, timeDelta)

        for action in d["actions"].values():
            if "children" in action and action["children"]:
                for child in action["children"][1:-1]:
                    self.stn.addConstraint(action["tStart"], self.actions[child]["tStart"], 2*timeDelta)
                    self.stn.addConstraint(self.actions[child]["tEnd"], action["tEnd"], 2*timeDelta)
                
                    logger.debug("Adding %s timedelta before %s (hierarchy)" % (action["tStart"], self.actions[child]["tStart"]))
                    logger.debug("Adding %s timedelta before %s (hierarchy)" % (self.actions[child]["tEnd"], action["tEnd"]))
                
                    if not self.stn.isConsistent():
                        logger.error("**  Error : invalid STN when importing the abstract links")
                        raise PlanImportError("invalid STN when importing the abstract links")

        if "absolute-time" in d:
            for time, value in d["absolute-time"]:
                
                t = time
                value = int(round(value*timeFactor))

                logger.debug("Adding %s at exactly %s" % (self.tpName[t], value))
                logger.debug("Bounds are %s" % self.stn.getBounds(self.tpName[t]))
                if not self.stn.mayBeConsistent(self.stn.getStartId(), self.tpName[t], value, value):
                    logger.error("**  Error : invalid STN when importing the plan and setting %s at %s" % (self.tpName[t], value))
                    raise PlanImportError("invalid STN when importing the plan and setting %s at %s" % (self.tpName[t], value))

                self.stn.addConstraint(self.stn.getStartId(), self.tpName[t], value, value)
                self.absTimes.append( (self.tpName[t], value) )
        
        if "current-time" in d:
            self.initTime = d["current-time"]
        else:
            self.initTime = None

        if "unavailable-actions" in d:
            for forbiddenAction in d["unavailable-actions"]:
                if [a["name"] for a in self.actions if a["name"] == forbiddenAction and not a["executed"]]:
                    logger.error("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)
                    raise PlanImportError("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)

        if not self.stn.isConsistent():
            logger.error("**  Error : invalid STN when importing the plan")
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
            if tp not in self.tpEnd.keys():
                tps.append((tp, self.stn.getBounds(tp).lb))
        
        tps.sort(key = lambda x:x[1])
        
        logger.debug("Nominal plan")
        for tp,time in tps:
            tpName = tp.split("-")[0]
            position = tp.split("-")[1]
            actionName = "-".join(tp.split("-")[2:])
            
            if position == "start":
                logger.debug("\t%5.2f (%s): %s" % (time/1000, tpName, actionName))

    def getLength(self):
        return self.stn.getLength()/timeFactor

    def setDeadline(self, deadline):
        self.stn.setHorizon(int(round(deadline*timeFactor)))
        
        if not self.stn.isConsistent():
            logger.warning("STN not valid after the set up of the deadline")
            return False
        return True
    
    def getJsonDescription(self):
        return deepcopy(self.jsonDescr)
    
    #assume value in ms
    def setTimePoint(self, tpName, value):
        logger.debug("plan.setTimpoint %s %s" % (tpName, value))
    
        if not "absolute-time" in self.jsonDescr:
            self.jsonDescr["absolute-time"] = []
            
        #add it in the plan description
        valueSec = float(value)/timeFactor
        self.jsonDescr["absolute-time"].append([int(tpName.split("-")[0]), valueSec])

        for index,action in self.jsonDescr["actions"].items():

            # The action starts with this tp
            if  int(tpName.split("-")[0]) == action["startTp"] and "dummy" not in action["name"]:
                action["executed"] = True
                action["locked"] = True
                
            #The action ends with this tp : update the action lengths
            if  int(tpName.split("-")[0]) == action["endTp"] and "dummy" not in action["name"]:
                if "dMax" in action:
                    start = self.stn.getBounds(str(self.actions[index]["tStart"]))
                    if start.ub != start.lb:
                        logger.error(tpName)
                        logger.error("Error : an action is executed but still has temporal flexibility %s,%s?" % (start.lb, start.ub))
                
                    duration = (value - start.lb)/timeFactor

                    action["dMax"] = max(duration, action["dMax"])
                
            self.jsonDescr["actions"][index] = action

        if not self.stn.mayBeConsistent(self.stn.getStartId(), tpName, value, value):
            logger.warning("Calling set timepoint for %s at %s" % (tpName, value))
            logger.warning("STN will not be consistent. Bonds are : %s" % self.stn.getBounds(tpName))
            logger.warning(self.stn.export())


        #add this constraint in the STN
        logger.debug("Executing %s at %s" % (tpName, value))
        return self.stn.addConstraint(self.stn.getStartId(), tpName, value, value)

    def setActionUnavailable(self, action):
        if not "unavailable-actions" in self.jsonDescr:
            self.jsonDescr["unavailable-actions"] = []
            
        self.jsonDescr["unavailable-actions"].append(action["name"])

    # Returns the plan with only actions for which the given agent is responsible 
    def getLocalJsonPlan(self, agent):
        data = self.getJsonDescription()
        
        keysToDelete = set()
        
        for k in list(data["actions"].keys()):
            if k in ["0", "1"]:
                continue #dummy init and dummy end
            
            action = data["actions"][k]
            
            if "agent" in action:
                actionAgent = action["agent"]
            else:
                logger.warning("Agent not defined for action %s" % action["name"])
                if action["name"].startswith("dummy"):
                    actionAgent = getAgentFromAction(action["name"].split(" ")[2:])
                else:
                    actionAgent = getAgentFromAction(action["name"].split(" "))
            
            if actionAgent != agent:
                #Remove this action from the plan
                keysToDelete.add(k)
                if "children" in action:
                    for c in action["children"]:
                        keysToDelete.add(c)
        
        logger.info("For %s, deleting %s" %(agent, keysToDelete))
        
        foreignTps = set([data["actions"][k]["startTp"] for k in keysToDelete]).union(set([data["actions"][k]["endTp"] for k in keysToDelete]))
        localTps = set([data["actions"][k]["startTp"] for k in data["actions"].keys() if k not in keysToDelete]).union(set([data["actions"][k]["endTp"] for k in data["actions"].keys() if k not in keysToDelete]))
        
        if(len(foreignTps.intersection(localTps)) > 0):
            logger.error("I got timepoints that are both foreign and local ?")
            logger.error(foreignTps.intersection(localTps))
        
        #deal together with both data
        for listKey in ["causal-links", "temporal-links"]:
            for i in reversed(range(len(data[listKey]))):
                tl = data[listKey][i]
                if tl["startTp"] in foreignTps and tl["endTp"] in foreignTps:
                    del data[listKey][i]
                elif tl["startTp"] in localTps and tl["endTp"] in localTps:
                    pass #keep it
                else:
                    #tricky : interaction between my action and another action
                    #only allowed if the foreign tp is in absoluteTime
                    if tl["startTp"] in foreignTps:
                        fTp,lTp = tl["startTp"],tl["endTp"]
                    else:
                        fTp,lTp = tl["endTp"],tl["startTp"]
                        
                    if lTp in [0,1]:
                        del data[listKey][i] # link with dummy init or dummy end
                    else:
                        pass #keep it

        for i in reversed(range(len(data["absolute-time"]))):
            t = data["absolute-time"][i][0]
            if t in foreignTps:
                del data["absolute-time"][i]

        for k in keysToDelete:
            #action = data["actions"][k]
            #data["causal-links"] =   [cl for cl in data["causal-links"]   if cl["startAction"] != k and cl["endAction"] != k]
            #data["temporal-links"] = [tl for tl in data["temporal-links"] if tl["startTp"] != action["startTp"] and tl["endTp"] != action["startTp"] and tl["endTp"] != action["startTp"] and tl["endTp"] != action["endTp"]]
            #data["absolute-time"] =  [pair for pair in data["absolute-time"] if pair[0] != action["startTp"] and pair[0] !=action["endTp"]]
            del data["actions"][k]

        #logger.info("local plan for %s : %s" % (agent,data))
        return data

    # get a dictionnary of plan with agent as a key.
    # Index of actions and tps are assumed to be in the same 'namespace'
    # special case for "dead" robots : remove their actions and the link using them
    @staticmethod
    def mergeJsonPlans(data):
        logger.warning(data)

        #Check for consistency
        for agent1,agent2 in itertools.combinations(data.keys(),2):
            actionKeys1 = set(data[agent1]["actions"].keys())
            actionKeys2 = set(data[agent2]["actions"].keys())
            s = actionKeys1.intersection(actionKeys2).difference(set(["0","1"]))
            if len(s) > 0:
                #Two agents are responsible for the same action ?
                logger.error("Agents %s and %s both uses the same action key" % (agent1, agent2))
                for k in s:
                    logger.error(data[agent1]["actions"][k])
                    logger.error(data[agent2]["actions"][k])
        
        #Get all the tp of each plan
        tps = set(itertools.chain.from_iterable((a["startTp"], a["endTp"]) for d in data.values() for a in d["actions"].values() ))
        
        for agent,d in data.items():
            for cl in d["causal-links"]:
                if cl["startTp"] not in tps or cl["endTp"] not in tps:
                    logger.error("Cannot find tp for %s" % cl)
            for tl in d["temporal-links"]:
                if tl["startTp"] not in tps or tl["endTp"] not in tps:
                    logger.error("Cannot find tp for %s" % tl)
            for tp,_ in d["absolute-time"]:
                if tp not in tps:
                    logger.error("Cannot find tp for absolute time %s" % tp)                   
        
        logger.info("Merging plans of %s" % " ".join(data.keys()))
        result = {}
        
        result["actions"] = {"0" : {"name":"dummy init", "dMax": 0.0, "dMin": 0.0, "endTp": 0,"startTp": 0},
                             "1" : {"name":"dummy end", "dMax": 0.0, "dMin": 0.0, "endTp": 1,"startTp": 1},
                            }
        result["causal-links"] = []
        result["temporal-links"] = []
        result["absolute-time"] = []

        for _,plan in data.items():
            for key,action in plan["actions"].items():
                if key in ["0", "1"]:
                    continue
            
                a = copy(action)
                result["actions"][key] = a
                
            for clink in plan["causal-links"]:
                cl = copy(clink)
                if cl not in result["causal-links"]:
                    result["causal-links"].append(cl)
                
            for tlink in plan["temporal-links"]:
                tl = copy(tlink)
                if tl not in result["temporal-links"]:
                    result["temporal-links"].append(tl)
                
            for tp,value in plan["absolute-time"]:
                result["absolute-time"].append([tp,value])
        
        return result
