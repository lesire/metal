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
    return actionName.split(" ")[0] in ["communicate"]

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
        d = Plan.preprocessPlanDict(d, agent)
        self.jsonDescr = deepcopy(d)

        self.mastnMsg = []

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
        agentsSet = set([a["agent"] for a in self.actions.values() if "agent" in a])
        self.tpEnd = {}
        if len(agentsSet) > 0:
            for a in agentsSet:
                if a != self.agent:
                    self.stn.addAgent(a)
                    
                self.tpEnd[a] = "1-end-" + a
                self.stn.addPoint(self.tpEnd[a], a) #end of all the actions of this robot

                self.stn.addPoint("1-endglobal-" + a, a) #end of all the actions of all the robots
            for a in agentsSet:
                for b in agentsSet:
                    self.stn.addConstraint("1-end-" + a, "1-endglobal-" + b, 1)

        self.tpName[1] = "1-end-" + self.agent if self.agent is not None else "1-end"

        #First compute the mapping tp/tpName
        for index,a in self.actions.items():
            if "dummy init" in a["name"] and a["name"] != "dummy init": continue #Ignore the dummy init of methods
            if "dummy end" in a["name"] and a["name"] != "dummy end": continue #Ignore the dummy end of methods
            
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

        # Then update the actions
        for index,a in self.actions.items():
            if "tStart" not in a: a["tStart"] = self.tpName[a["startTp"]]
            if "tEnd" not in a: a["tEnd"] = self.tpName[a["endTp"]]
            
            if isActionControllable(a["name"]):
                a["controllable"] = True
            else:
                a["controllable"] = False
            
            if "children" in a and a["children"]:
                a["abstract"] = True
                a["controllable"] = True
            else:
                a["abstract"] = False
                
            if not "executed" in a:
                a["executed"] = False
            if not "locked" in a:
                a["locked"] = False
            a["dMin"] = abs(a["dMin"])
            
            self.actions[index] = a

        for a in self.actions.values():

            if self.agent is not None and "agent" in a:
                endPoint = "1-end-" + a["agent"]
            else:
                endPoint = "1-end"

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
                self.stn.addConstraint(a["tEnd"], endPoint, 1)  #prevent the end to be executed before any actions (for instance if the last action does not provide a goal)
            
            if a["tStart"] != a["tEnd"] and not a["controllable"]:
                if "dMin" in a:
                    dMin = int(round(timeFactor*a["dMin"]))
                    
                    if "communicate-meta" in a["name"]:
                        dMin = 0 #in this case a problem can arise if the action was in fact shorter, we did not get the end time but another robot executed a action depending of it and notifies us. This is a hack for a lack of a better solution
                    
                    self.stn.addConstraint(a["tStart"], a["tEnd"], dMin)
                else:
                    logger.warning("Action %s does not have a dMin ?" % a["name"])
      
            elif a["tStart"] != a["tEnd"]:
                    self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), 1)

                #if any([re.match(regex, a["name"]) for regex in nonRandomAction]):
                #     self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(timeFactor*a["dMin"]), int(timeFactor*a["dMin"]))

        for cl in d["causal-links"]:
            start = self.tpName[cl["startTp"]]

            # If end point is the endTp, then get the one corresponding to the agent executing the action
            end = self.tpName[cl["endTp"]]
            if cl["endTp"] == 1:
                agent = self.tpAgent[cl["startTp"]]
                end = self.tpEnd[agent]
            
            if start.startswith("0-"):
                self.stn.addConstraint(self.stn.getStartId(), end, timeDelta)
            else:
                self.stn.addConstraint(start, end, timeDelta)

        for tl in d["temporal-links"]:
            startTp = self.tpName[tl["startTp"]]
            endTp = self.tpName[tl["endTp"]]
            if startTp.startswith("0-"):
                startTp = self.stn.getStartId()

            lb = tl.get("lb", timeDelta)
            if "ub" not in tl:
                self.stn.addConstraint(startTp, endTp, lb)
            else:
                self.stn.addConstraint(startTp, endTp, lb, tl["ub"])

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
                
                #Ignore absolute dates on communication if there are in the future to allow their exectution
                #as soon as possible.
                if "start-communicate" in self.tpName[time]:
                    if "current-time" not in d or value > d["current-time"]:
                        self.jsonDescr["absolute-time"].remove([time, value])
                        logger.info("Removing the absolute time of %s" % self.tpName[time])
                        continue

                t = time
                value = int(round(value*timeFactor))
                
                #potentialActions = [a for a in self.actions.values() if a["endTp"] == t]
                #if len(potentialActions) > 0 and any(map(lambda x: not x["controllable"] and not x["executed"] and not x["abstract"], potentialActions)):
                #    logger.error("Plan with an absolute time for an uncontrollable, non-executed action")
                #    logger.error(self.tpName[t])
                #    continue

                logger.debug("Adding %s at exactly %s" % (self.tpName[t], value))
                logger.debug("Bounds are %s" % self.stn.getBounds(self.tpName[t]))
                if not self.stn.mayBeConsistent(self.stn.getStartId(), self.tpName[t], value, value):
                    logger.error("**  Error : invalid STN when importing the plan and setting %s at %s" % (self.tpName[t], value))
                    logger.error("Bounds are %s" % self.stn.getBounds(self.tpName[t]))
                    #logger.error("Stn is %s" % self.stn.export())
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
            elif tp.startswith("1-end"):
                logger.debug("\t%5.2f (%s): %s" % (time/1000, tpName, tp))

        self.ids = d["ID"]["parents"] + [d["ID"]["value"]]

    # Called before the plan is used.
    # Do some checks and modification of the initial plan.
    @staticmethod
    def preprocessPlanDict(data, agent):
        for a in data["actions"].values():
            if "agent" not in a:
                if a["name"] == "dummy init" or a["name"] == "dummy end":
                    a["agent"] = agent
                else:
                    logger.error("Processing a plan with no agent defined for action : %s" % a["name"])

        for a in data["actions"].values():
            if isActionControllable(a["name"]):
                if "dMax" in a:
                    del a["dMax"]

        if "absolute-time" in data:
            for a in data["actions"].values():
                if "communicate" in a["name"] and not a.get("executed", False): #remove deadline on past communicate action
                    for i in reversed(range(len(data["absolute-time"]))):
                        t = data["absolute-time"][i][0]
                        if t == a["startTp"] or t == a["endTp"]:
                            a["locked"] = True
                            del data["absolute-time"][i]
    
        if "ID" not in data:
            data["ID"] = {"value":0, "parents":[]}
    
        return data
    

    def getLength(self):
        return self.stn.getLength()/timeFactor
    
    def getJsonDescription(self, currentTime = None):
        result = deepcopy(self.jsonDescr)
        
        if "absolute-time" in result:
            executedTps = {l[0]:l[1] for l in result["absolute-time"]}
            
            for a in result["actions"].values():
                if a["startTp"] in executedTps and a["endTp"] in executedTps:
                    if "dMin" in a:
                        a["dMin"] = min(abs(a["dMin"]), (executedTps[a["endTp"]] - executedTps[a["startTp"]])/timeFactor)
                    if "dMax" in a:
                        a["dMax"] = max(abs(a["dMax"]), (executedTps[a["endTp"]] - executedTps[a["startTp"]])/timeFactor)
        
        if currentTime is not None:
            result["current-time"] = currentTime
        return result
    
    # Add a temporal constraint to the plan.
    # Will be exported into Json to be included during the repair attemps
    def addTemporalConstraint(self, startTp, endTp, lb, ub = None, cbStnUpdated = (lambda x: logger.error("toto"))):
        if startTp is None:
            startTp = self.stn.getStartId()
            startTpIndex = 0
        else:
            startTpIndex = int(startTp.split("-")[0])

        if ub is None:
            l = self.stn.addConstraint(startTp, endTp, lb)
            cbStnUpdated({"type":"add", "start" : startTp, "end" : endTp, "lb" : lb})
        else:
            l = self.stn.addConstraint(startTp, endTp, lb, ub)
            cbStnUpdated({"type":"add", "start" : startTp, "end" : endTp, "lb" : lb, "ub" :ub})


        self.mastnMsg = l + self.mastnMsg

        if not self.stn.isConsistent():
            logger.warning("When adding constraint between %s and %s (%s, %s), STN became inconsistent" % (startTp, endTp, lb, ub))

        tLink = {"startTp":startTpIndex, "endTp":int(endTp.split("-")[0]), "lb":lb}
        if ub is not None:
            tLink["ub"] = ub

        logger.info("Adding %s to the plan json description" % tLink)
        self.jsonDescr["temporal-links"].append(tLink)

    #assume value in ms
    def setTimePoint(self, tpName, value, cbStnUpdated = (lambda x: logger.error("toto"))):
        logger.debug("plan.setTimpoint %s %s" % (tpName, value))
    
        c = self.stn.getBounds(tpName)
        if value < c.lb or value > c.ub:
        #if not self.stn.mayBeConsistent(self.stn.getStartId(), tpName, value, value):
            logger.warning("Calling set timepoint for %s at %s" % (tpName, value))
            logger.warning("STN will not be consistent. Bonds are : %s" % self.stn.getBounds(tpName))
            #logger.warning(self.stn.export())


        #add this constraint in the STN
        logger.info("Executing %s at %s. Bounds are %s" % (tpName, value, c))
        l = self.stn.addConstraint(self.stn.getStartId(), tpName, value, value)
        cbStnUpdated({"type":"add", "start" : self.stn.getStartId(), "end" : tpName, "lb" : value, "ub" : value})
        
        self.mastnMsg = self.mastnMsg + l

        if not self.stn.isConsistent():
            logger.error("When executing %s at %s, stn became inconsistent" % (tpName, value))
            logger.error("Bounds were : %s,%s" % (c.lb, c.ub))


        if not "absolute-time" in self.jsonDescr:
            self.jsonDescr["absolute-time"] = []
            
        #add it in the plan description.
        if not tpName.startswith("1-end-"): #ignore the end of the local plan. Only the global end matters
            tpIndex = int(tpName.split("-")[0])
            valueSec = float(value)/timeFactor
            # For communicate action, the Json plan can have a deadline that we ignored. We need to rewrite it with the real execution time
            if tpIndex in [t for t,_ in self.jsonDescr["absolute-time"]]:
                if "start-communicate" not in tpName:
                    logger.error("Trying to execute %s. But its date is already set in the json description. Overwriting it" % tpName)
                self.jsonDescr["absolute-time"] = [(t,v) for t,v in self.jsonDescr["absolute-time"] if t != tpIndex] #remove the previous absolute time if needed.
            self.jsonDescr["absolute-time"].append([tpIndex, valueSec])

        for index,action in self.jsonDescr["actions"].items():

            # The action starts with this tp
            if  int(tpName.split("-")[0]) == action["startTp"] and "dummy" not in action["name"]:
                action["executed"] = True
                action["locked"] = True
                
            #The action ends with this tp : update the action lengths
            if  int(tpName.split("-")[0]) == action["endTp"] and "dummy" not in action["name"]:
                if "dMax" in action and self.stn.isConsistent():
                    start = self.stn.getBounds(str(self.actions[index]["tStart"]))
                    if start.ub != start.lb:
                        logger.error(self.actions[index]["tStart"])
                        logger.error("Error : an action is executed but still has temporal flexibility %s,%s?" % (start.lb, start.ub))
                
                    duration = (value - start.lb)/timeFactor

                    action["dMax"] = max(duration, action["dMax"])
                
            self.jsonDescr["actions"][index] = action

    def getMastnMsg(self):
        l = self.mastnMsg
        self.mastnMsg = []
        return l

    def setActionUnavailable(self, action):
        if not "unavailable-actions" in self.jsonDescr:
            self.jsonDescr["unavailable-actions"] = []
            
        self.jsonDescr["unavailable-actions"].append(action["name"])

    def setForeignActionExecuted(self, key, agentFrom):
        if key not in self.jsonDescr["actions"]:
            logger.error("Received an update from %s with executed actions %k. I do not know it" % (agentFrom, key))
        elif self.jsonDescr["actions"][key]["agent"] != agentFrom:
            logger.error("%s pretends he has executed %k. But this action belongs to %s" % (agentFrom, key, self.jsonDescr["actions"][key]["agent"]))
        else:
            if "executed" not in self.jsonDescr["actions"][key] or not self.jsonDescr["actions"][key]["executed"]:
                logger.info("I'm informed that %s has executed %s" % (agentFrom, self.jsonDescr["actions"][key]["name"]))
                self.jsonDescr["actions"][key]["executed"] = True
                self.jsonDescr["actions"][key]["locked"] = True
    
    
    def setForeignTp(self, tpName, value):
        tp = tpName.split("-")[0]
        try:
            tp = int(tp)
        except:
            logger.error("Cannot convert %s from %t to a tpNumber" % tp)
            return

        for k,v in self.jsonDescr["absolute-time"]:
            vMs = int(v*timeFactor + 0.5)
            if k == tp and int(value) != vMs:
                logger.error("setForeignTp called with an already executed point %s. %s %s! " % (tpName, value/timeFactor, v))
                logger.error("%s %s %s" % (int(value), vMs, int(value) != vMs))
                return
            elif k == tp and int(value) == vMs:
                return

        found = any([tp == a["startTp"] or tp == a["endTp"] for a in self.jsonDescr["actions"].values()])
        if not found:
            logger.error("I'm informed of the execution of %s. But it is not associated to any action in my plan" % tpName)
            return

        logger.info("I'm informed that the timepoint %s was executed at %s" % (tpName, value/timeFactor))
        self.jsonDescr["absolute-time"].append([tp, value/timeFactor])

    # Returns the plan with only actions for which the given agent is responsible 
    def getLocalJsonPlan(self, agent, currentTime = None):
        data = self.getJsonDescription(currentTime=currentTime)
        
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

    # get a dictionnary of plan with agent as a key. Given plan sould be local
    # Index of actions and tps are assumed to be in the same 'namespace'
    # special case for "dead" robots : remove their actions and the link using them
    @staticmethod
    def mergeJsonPlans(data, idAgent=None):
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
        
        if idAgent is None:
            result["ID"] = data[list(data.keys()[0])]["ID"] # take a random ID among the plans
        else:
            result["ID"] = data[idAgent]["ID"]
        
        return result

    # get a plan (as a dictionnary) and remove an action from it.
    # Remove all the relevant element to keep the plan consistent
    @staticmethod
    def removeAction(planJson, actionKey):
        planJson = deepcopy(planJson)
        a = planJson["actions"][actionKey]
        
        deletedTps = [a["startTp"], a["endTp"]]

        # must remove all elements that refer to the deleted action
        del planJson["actions"][actionKey]
        planJson["causal-links"] = [cl for cl in planJson["causal-links"] if cl["startTp"] not in deletedTps and cl["endTp"] not in deletedTps]
        planJson["temporal-links"] = [cl for cl in planJson["temporal-links"] if cl["startTp"] not in deletedTps and cl["endTp"] not in deletedTps]
        planJson["absolute-time"] = [(t,v) for t,v in planJson["absolute-time"] if t not in deletedTps]

        return planJson


