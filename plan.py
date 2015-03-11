from __future__ import division

from copy import copy
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

timeDelta = 1
timeFactor = 1000 # all float duration are multiplied by this and then cast into int

class PlanImportError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


class Plan:
    def __init__(self, planStr):
        d = json.loads(planStr)
        self.jsonDescr = d

        self.stn = pystn.STNInt(pystn.StnType.IPPCSTN)
        
        self.tpName = {}
        self.tpName[1] = "1-end"
        self.stn.addPoint("1-end")
        
        self.actions = copy(d["actions"])
        for index,a in d["actions"].items():
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
            else:
                a["abstract"] = False
                
            if not "executed" in a:
                a["executed"] = False
            if not "locked" in a:
                a["locked"] = False
            a["dMin"] = abs(a["dMin"])
            
            self.actions[index] = a
        
        for a in self.actions.values():

            if a["tStart"][0] != "0" and a["tStart"] not in self.stn.getNodeIds():
                self.stn.addPoint(a["tStart"])
                self.stn.addConstraint(a["tStart"], self.tpName[1], 0)
            if a["tEnd"][0] != "0" and a["tEnd"] not in self.stn.getNodeIds():
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

        for cl in d["causal-links"]:            
            start = self.tpName[cl["startTp"]]
            end = self.tpName[cl["endTp"]]

            if cl["startTp"] == 0:
                self.stn.addConstraint(self.stn.getStartId(), end, timeDelta)
            else:
                self.stn.addConstraint(start, end, timeDelta)
                        
        for tl in d["temporal-links"]:
            start = self.tpName[tl["startTp"]]
            end = self.tpName[tl["endTp"]]
            
            if tl["startTp"] == 0:
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
                
                logging.debug("Adding %s at exactly %s" % (self.tpName[time], value))
                logging.debug("Bounds are %s" % self.stn.getBounds(self.tpName[time]))
                if not self.stn.mayBeConsistent(self.stn.getStartId(), self.tpName[time], value, value):
                    logging.error("**  Error : invalid STN when importing the plan and setting %s at %s" % (self.tpName[time], value))
                    raise PlanImportError("invalid STN when importing the plan and setting %s at %s" % (self.tpName[time], value))

                self.stn.addConstraint(self.stn.getStartId(), self.tpName[time], value, value)

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
            
            
    def getLength(self):
        return self.stn.getLength()/timeFactor

    def setDeadline(self, deadline):
        self.stn.setHorizon(int(round(deadline*timeFactor)))
        
        if not self.stn.isConsistent():
            logging.warning("STN not valid after the set up of the deadline")
            return False
        return True
    
    def getJsonDescription(self):
        return self.jsonDescr
    
    #assume value in ms
    def setTimePoint(self, tpName, value):
        if not "absolute-time" in self.jsonDescr:
            self.jsonDescr["absolute-time"] = []

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

        #add it in the plan description
        value = float(value)/timeFactor
        self.jsonDescr["absolute-time"].append([int(tpName.split("-")[0]), value])

        logging.debug("Executing %s at %s" % (tpName, value))

    def setActionUnavailable(self, action):
        if not "unavailable-actions" in self.jsonDescr:
            self.jsonDescr["unavailable-actions"] = []
            
        self.jsonDescr["unavailable-actions"].append(action["name"])
