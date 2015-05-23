import logging; logger = logging.getLogger("hidden")

import time
from functools import partial
import os

class AbstractActionExecutor:

    def __init__(self, **kwargs):
        pass

    def create_or_clean(self, folder):
        if not os.path.exists(folder):
            os.makedirs(folder)
        for f in os.listdir(folder):
            os.remove(os.path.join(folder, f))

    def execute(self, action, cb):
        a, *args = action["name"].split(" ")
        a = a.replace('-', '_')
        logger.debug("executing {a}({args})".format(a=a,args=args))
        method = getattr(self, a)
        method(*args, cb=partial(self.report, action, cb), actionJson=action)

    def stop(self, action):
        if not action["controllable"]:
            logger.error("Stopping an uncontrollable action : %s" % action)

        if "_stop" in dir(self):
            self._stop(action)
        else:
            logger.info("Stopping %s" % action["name"])

        pass

    def report(self, action, cb, report):
        logger.info("Reporting for action {a}: {r}".format(a=action["name"],r=report))
        cb(action, report=report)
        
    #called periodically
    def update(self):
        pass
    
class DummyActionExecutor(AbstractActionExecutor):
    _name = "dummy"

    def __init__(self, agentName=None, **kwargs):
        logger.info("Using dummy executor")
        self.nextEvents = [] # list of dictionnary with time/callback to call
        
        self.agent = agentName
        self.pos = {}
        
    def init(self, who, a, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logger.info("Init {w} at {a} in {d}".format(w=who,a=a,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append({"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
      
    def move(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logger.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        self.pos[who] = b

        dur = actionJson["dMin"]
        logger.info("moving {w} from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def observe(self, who, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who):
            logger.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(who in self.pos and self.pos[who] != a):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who], a))

        dur = actionJson["dMin"]
        logger.info("{w} observe from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})

    def communicate(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logger.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logger.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logger.info("Com between {w1} at {a} with {w2} at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        #currentTime = time.time()
        #self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})

    def has_communicated(self, *args, **kwargs):
        pass
    
    def communicate_meta(self, *args, **kwargs):
        pass
    
    def track(self, *args, **kwargs):
        logger.info("Received a track action")
    
    def update(self):
        currentTime = time.time()

        for d in [d for d in self.nextEvents if d["time"] <= currentTime]:
            #if "move mana" in d["actionJson"]["name"]:
            #    d["cb"]({"type":"target_found", "position":{"x":1,"y":1}})
            #    logger.warning("Simulating target found")
            #else:
            d["cb"]({"type":"ok"})
            logger.info("calling a callback for %s" % d["actionJson"]["name"])
            
        self.nextEvents = [d for d in self.nextEvents if d["time"] > currentTime]
    
class DummyMAActionExecutor(DummyActionExecutor):
    _name = "dummy-ma"

    def __init__(self, agentName, folder, **kwargs):
        self.folder = folder
        self.create_or_clean(folder)
        self.agent = agentName
        self.pos = {}
        
        logger.info("Using dummy MA executor. Writing files in " + self.folder)
        self.nextEvents = [] # list of dictionnary with time/callback to call
        
        self.inCom = []

    def communicate_aav_agv(self, who1, who2, a, b, cb, actionJson, **kwargs):
        if(self.agent is not None and self.agent != who1 and self.agent != who2):
            logger.warning("Executor for %s ask to execute action %s" % (self.agent, actionJson["name"]))

        if(self.agent is not None and "agent" in actionJson and self.agent != actionJson["agent"]):
            logger.warning("Executor for %s ask to execute the split communicate action %s for %s" % (self.agent, actionJson["name"], actionJson["agent"]))

        if(who1 in self.pos and self.pos[who1] != a):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who1], a))
        if(who2 in self.pos and self.pos[who2] != b):
            logger.warning("Lauching a move action from %s but the last recorded pos is %s" %(self.pos[who2], b))

        dur = actionJson["dMin"]
        logger.info("Com between {w1} at {a} with {w2} at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        filename = os.path.join(self.folder, "communicate_{w1}_{w2}_{a}_{b}_{agent}".format(w1=who1,w2=who2,a=a,b=b,agent=self.agent))
        if os.access(filename, os.R_OK):
            logger.error("Synchronisation file already created ?!? : %s" % filename)
        else:
            fd = os.open(filename, os.O_CREAT | os.O_EXCL | os.O_RDWR)
            os.close(fd)
            
        self.inCom.append("communicate {w1} {w2} {a} {b}".format(w1=who1,w2=who2,a=a,b=b))

    def has_communicated(self, *args, **kwargs):
        pass
    
    def _stop(self, action):
        name = action["name"]
        
        if "has-communicated" in name:
            return
        
        if name not in self.inCom:
            logger.error("End of %s. But it was never started" % name)
            logger.error("Started coms : %s" % self.inCom)
            logger.error(self.inCom[0] == name)
            logger.error(type(self.inCom[0]))
            logger.error(type(name))
            logger.error(self.inCom[0])
            logger.error(name)
        else:    
            agents = name.split(" ")[1:3]
            filename1 = os.path.join(self.folder, name.replace(" ","_") + "_" + agents[0])
            filename2 = os.path.join(self.folder, name.replace(" ","_") + "_" + agents[1])
            result = [os.access(filename1, os.F_OK), os.access(filename2, os.F_OK)]
            if not all(result):
                logger.error("Error in coms at the end of %s" % name)
                if not result[0]:
                    logger.error("\t%s was not here" % agents[0])
                if not result[1]:
                    logger.error("\t%s was not here" % agents[1])
            else:
                logger.info("End of com %s : everyone was here" % name)
                
                    
            del self.inCom[self.inCom.index(name)]


class DummyDelay(DummyActionExecutor):
    _name = "delay"

    def move(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        if who == "ressac2" and a == "pt_aav_22229_-592" and b == "pt_aav_18235_-6582":
            dur = dur + 20
            logger.warning("Delaying action %s" % actionJson["name"])
        if who == "mana":
            dur = dur + 30
            logger.warning("Delaying action %s" % actionJson["name"])
        if who == "effibot1":
            if "inc" not in dir(self):
                self.inc = 0
            self.inc += 1
            
            if self.inc < 3:
                dur = dur + 10
                logger.warning("Delaying action %s" % actionJson["name"])
        if who == "r1":
            dur = dur + 1
            logger.warning("Delaying action %s" % actionJson["name"])
        logger.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + dur),"cb": cb, "actionJson":actionJson})
        
class DummyDelayMA(DummyMAActionExecutor):
    _name = "delay-ma"

    def move(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        if who == "ressac2" and a == "pt_aav_22229_-592" and b == "pt_aav_18235_-6582":
            dur = dur + 20
            logger.warning("Delaying action %s" % actionJson["name"])
        logger.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + dur),"cb": cb, "actionJson":actionJson})

if __name__=="__main__":
    e = DummyMAActionExecutor("ressac1", "/tmp/hidden2")
    #e.communicate_aav_aav("ressac1", "ressac2", "pt1", "pt2", None, {"dMin":1})
    e.execute({"name" : "communicate-aav-aav ressac1 ressac2 pt1 pt2", "dMin":1}, None)
    e._stop({"name":"communicate-aav-aav ressac1 ressac2 pt1 pt2"})
