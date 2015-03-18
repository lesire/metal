import logging
import re
import time
from functools import partial
import os

class AbstractActionExecutor:
    def __init__(self):
        pass

    def execute(self, action, cb):
        a, *args = action["name"].split(" ")
        a = a.replace('-', '_')
        logging.debug("executing {a}({args})".format(a=a,args=args))
        method = getattr(self, a)
        method(*args, cb=partial(self.report, action, cb), actionJson=action)

    def stop(self, action):
        if not action["controllable"]:
            logging.error("Stopping an uncontrollable action : %s" % action)

        if "_stop" in dir(self):
            self._stop(action)
        else:
            logging.info("Stopping %s" % action["name"])

        pass

    def report(self, action, cb, report):
        logging.info("Reporting for action {a}: {r}".format(a=action["name"],r=report))
        cb(action)
        
    #called periodically
    def update(self):
        pass

class DummyActionExecutor(AbstractActionExecutor):
    def __init__(self, agentName=None):
        logging.info("Using dummy executor")
        self.nextEvents = [] # list of dictionnary with time/callback to call
        
        self.agent = agentName
        self.pos = {}
        
    def move_aav(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        self.pos[who] = b

        dur = actionJson["dMin"]
        logging.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def observe_agv(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        dur = actionJson["dMin"]
        logging.info("{w} observe from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def move_agv(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        self.pos[who] = b
        
        dur = actionJson["dMin"]
        logging.info("moving {w}(agv) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def observe_aav(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        dur = actionJson["dMin"]
        logging.info("{w} observe from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def communicate_aav_agv(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logging.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(agv) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        #currentTime = time.time()
        #self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def communicate_aav_aav(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logging.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(aav) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        #currentTime = time.time()
        #self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
    
    def has_communicated(self, *args, **kwargs):
        pass
        
    def update(self):
        currentTime = time.time()

        for d in [d for d in self.nextEvents if d["time"] <= currentTime]:
            d["cb"]("ok")
            logging.info("calling a callback for %s" % d["actionJson"]["name"])
            
        self.nextEvents = [d for d in self.nextEvents if d["time"] > currentTime]
    
class DummyMAActionExecutor(DummyActionExecutor):
    def __init__(self, agentName, folder):
        self.folder = folder
        self.agent = agentName
        self.pos = {}
        
        logging.info("Using dummy MA executor. Writing files in " + self.folder)
        self.nextEvents = [] # list of dictionnary with time/callback to call
        
        self.inCom = []

    def communicate_aav_agv(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logging.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(agv) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        filename = os.path.join(self.folder, "communicate-aav-agv_{w1}_{w2}_{a}_{b}_{agent}".format(w1=who1,w2=who2,a=a,b=b,agent=self.agent))
        if os.access(filename, os.R_OK):
            logging.error("Synchronisation file already created ?!? : %s" % filename)
        else:
            fd = os.open(filename, os.O_CREAT | os.O_EXCL | os.O_RDWR)
            os.close(fd)
            
        self.inCom.append("communicate-aav-agv {w1} {w2} {a} {b}".format(w1=who1,w2=who2,a=a,b=b))

    def communicate_aav_aav(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logging.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logging.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logging.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(aav) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        filename = os.path.join(self.folder, "communicate-aav-aav_{w1}_{w2}_{a}_{b}_{agent}".format(w1=who1,w2=who2,a=a,b=b,agent=self.agent))
        if os.access(filename, os.R_OK):
            logging.error("Synchronisation file already created ?!? : %s" % filename)
        else:
            fd = os.open(filename, os.O_CREAT | os.O_EXCL | os.O_RDWR)
            os.close(fd)
            
        self.inCom.append("communicate-aav-aav {w1} {w2} {a} {b}".format(w1=who1,w2=who2,a=a,b=b))
        
    def has_communicated(self, *args, **kwargs):
        pass
    
    def _stop(self, action):
        name = action["name"]
        
        if "has-communicated" in name:
            return
        
        if name not in self.inCom:
            logging.error("End of %s. But it was never started" % name)
            logging.error("Started coms : %s" % self.inCom)
            logging.error(self.inCom[0] == name)
            logging.error(type(self.inCom[0]))
            logging.error(type(name))
            logging.error(self.inCom[0])
            logging.error(name)
        else:    
            agents = name.split(" ")[1:3]
            filename1 = os.path.join(self.folder, name.replace(" ","_") + "_" + agents[0])
            filename2 = os.path.join(self.folder, name.replace(" ","_") + "_" + agents[1])
            result = [os.access(filename1, os.F_OK), os.access(filename2, os.F_OK)]
            if not all(result):
                logging.error("Error in coms at the end of %s" % name)
                if not result[0]:
                    logging.error("\t%s was not here" % agents[0])
                if not result[1]:
                    logging.error("\t%s was not here" % agents[1])
            else:
                logging.info("End of com %s : everyone was here" % name)
                
                    
            del self.inCom[self.inCom.index(name)]


class DummyDelay(DummyActionExecutor):
    def move_aav(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        if who == "ressac2" and a == "pt_aav_22229_-592" and b == "pt_aav_18235_-6582":
            dur = dur + 20
            logging.warning("Delaying action %s" % actionJson["name"])
        logging.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + dur),"cb": cb, "actionJson":actionJson})
        
class DummyDelayMA(DummyMAActionExecutor):
    def move_aav(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        if who == "ressac2" and a == "pt_aav_22229_-592" and b == "pt_aav_18235_-6582":
            dur = dur + 20
            logging.warning("Delaying action %s" % actionJson["name"])
        logging.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + dur),"cb": cb, "actionJson":actionJson})

if __name__=="__main__":
    e = DummyMAActionExecutor("ressac1", "/tmp/hidden2")
    #e.communicate_aav_aav("ressac1", "ressac2", "pt1", "pt2", None, {"dMin":1})
    e.execute({"name" : "communicate-aav-aav ressac1 ressac2 pt1 pt2", "dMin":1}, None)
    e._stop({"name":"communicate-aav-aav ressac1 ressac2 pt1 pt2"})
