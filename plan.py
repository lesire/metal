from __future__ import division

import json
import logging
import sys

try:
    import pystn
except ImportError:
    print("[error] cannot import pystn")
    sys.exit(1)


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
        
        self.actions = []
        for a in d["actions"]:
            if a[1] not in self.tpName:
                self.tpName[a[1]] = str(str(a[1]) + "-start-" + a[0])
            if a[2] not in self.tpName:
                self.tpName[a[2]] = str(str(a[2]) + "-end-" + a[0])
            self.actions.append({"name":a[0], "tStart":self.tpName[a[1]], "tEnd":self.tpName[a[2]]})
            if len(a) > 3 and a[3] != None:
                self.actions[-1]["dMin"] = abs(-a[3])
            if len(a) > 4 and a[4] != None:
                self.actions[-1]["dMax"] = a[4]
            else:
                self.actions[-1]["dMax"] = self.actions[-1]["dMin"]
            if len(a) > 5 and a[5] != None and a[5]:
                self.actions[-1]["executed"] = True
            else:
                self.actions[-1]["executed"] = False
        
        for a in self.actions:
            if a["tStart"][0] != "0" and a["tStart"] not in self.stn.getNodeIds():
                self.stn.addPoint(a["tStart"])
                self.stn.addConstraint(a["tStart"], self.tpName[1], 0)
            if a["tEnd"][0] != "0" and a["tEnd"] not in self.stn.getNodeIds():
                self.stn.addPoint(a["tEnd"])
                self.stn.addConstraint(a["tEnd"], self.tpName[1], 0)
            
            if a["tStart"] != a["tEnd"]:
                if "dMin" in a:
                    self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(round(timeFactor*a["dMin"])))

                #if any([re.match(regex, a["name"]) for regex in nonRandomAction]):
                #     self.stn.addConstraint(str(a["tStart"]), str(a["tEnd"]), int(timeFactor*a["dMin"]), int(timeFactor*a["dMin"]))

        # 1 is the end point, should be after every one else
        for node in self.stn.getNodeIds():
            if node != self.tpName[1]:
                self.stn.addConstraint(node, self.tpName[1], timeDelta)

        for cl in d["causal-links"]:
            UNDEFINED = 0
            ATSTART = 1
            ATEND = 2
            OVERALL = 3
            UNTIMED = 4
            
            if cl[2] == ATSTART:
                start = self.actions[cl[0]]["tStart"]
            else:
                start = self.actions[cl[0]]["tEnd"]
                
            if cl[3] in [UNDEFINED, ATSTART, OVERALL, UNTIMED]:
                end = self.actions[cl[1]]["tStart"]
            else:
                end = self.actions[cl[1]]["tEnd"]

            if cl[0] == 0:
                self.stn.addConstraint(self.stn.getStartId(), end, timeDelta)
            else:
                self.stn.addConstraint(start, end, timeDelta)
                        
        for tl in d["temporal-links"]:
            if tl[0] == 0:
                self.stn.addConstraint(self.stn.getStartId(), self.tpName[tl[1]], timeDelta)
            else:
                self.stn.addConstraint(self.tpName[tl[0]], self.tpName[tl[1]], timeDelta)

        for am in d["abstract-mapping"]:
            self.actions[am[0]]["abstract"] = True
        
            if not "dummy" in self.actions[am[1]]["name"]:
                #factor 2 to account for the double causal link ?
                self.stn.addConstraint(self.actions[am[0]]["tStart"], self.actions[am[1]]["tStart"], 2*timeDelta)
                self.stn.addConstraint(self.actions[am[1]]["tEnd"], self.actions[am[0]]["tEnd"], 2*timeDelta)
                
                logging.debug("Adding %s timedelta before %s" % (self.actions[am[1]]["tEnd"], self.actions[am[0]]["tEnd"]))
                logging.debug("bounds : %s,%s" % (self.stn.getConstraint(self.actions[am[1]]["tEnd"], self.actions[am[0]]["tEnd"]).lb, self.stn.getConstraint(self.actions[am[1]]["tEnd"], self.actions[am[0]]["tEnd"]).ub))
                
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
                    print("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)
                    raise PlanImportError("** ERROR : a given plan has a forbidden action : %s" % forbiddenAction)

        if not self.stn.isConsistent():
            logging.error("**  Error : invalid STN when importing the plan")
            raise PlanImportError("invalid STN when importing the plan")
        
        #remove dummy init/end actions
        self.actions = [a for a in self.actions if not ("dummy" in a["name"] and a["name"] != "dummy init" and a["name"] != "dummy end")]

    def getLength(self):
        return self.stn.getLength()/timeFactor

    def setDeadline(self, deadline):
        self.stn.setHorizon(int(round(deadline*timeFactor)))
        
        if not self.stn.isConsistent():
            logging.warning("STN not valid after the set up of the deadline")
            return False
        return True
    
    def getJsonDescription(self):
        #fill the agents name if necessary
        for a in self.jsonDescr["actions"]:
            if len(a) == 5:
                a.append(False) #executed
            if len(a) == 6:
                a.append(a[5]) #locked

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

        actions = filter(lambda a: a[1] == int(tpName.split("-")[0]) and a[1]>1 and "dummy" not in a[0], self.jsonDescr["actions"]) #actions that start with this tp
        for a in actions:
            if len(a) >= 6:
                a[5] = True #mark as executed
                a[6] = True #mark as locked
            else:
                a.append(True)

        #update the action lengths
        actions = filter(lambda a: a[2] == int(tpName.split("-")[0]) and a[2]>1 and "dummy" not in a[0], self.jsonDescr["actions"]) #actions that end with this tp
        for a in actions:
            if a[4] is not None:
                start = self.stn.getConstraint(self.stn.getStartId(), str(a[1]) + "-start-" + str(a[0]))
                #c = self.stn.getConstraint(str(a[1]) + "-start-" + str(a[0]), str(a[2]) + "-end-" + str(a[0]))
                if start.ub != start.lb:
                    print("Error : an action is executed but still has temporal flexibility %s,%s?" % (start.lb, start.ub))
                    sys.exit(1)
                    return
                
                duration = (value - start.lb)/timeFactor

                a[4] = max(duration, a[4])

        #add it in the plan description
        value = float(value)/timeFactor
        self.jsonDescr["absolute-time"].append([int(tpName.split("-")[0]), value])

        logging.debug("Executing %s at %s" % (tpName, value))

    def setActionUnavailable(self, action):
        if not "unavailable-actions" in self.jsonDescr:
            self.jsonDescr["unavailable-actions"] = []
            
        self.jsonDescr["unavailable-actions"].append(action["name"])
