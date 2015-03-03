import logging
import re
import time
from functools import partial

class AbstractActionExecutor:
    def __init__(self):
        pass

    def execute(self, action, cb):
        a, *args = action["name"].split(" ")
        logging.debug("executing {a}({args})".format(a=a,args=args))
        method = getattr(self, a.replace('-', '_'))
        method(*args, cb=partial(self.report, action, cb), actionJson=action)

    def report(self, action, cb, report):
        logging.info("reporting for action {a}: {r}".format(a=action["name"],r=report))
        cb(action)
        
    #called periodically
    def update(self):
        pass

class DummyActionExecutor(AbstractActionExecutor):
    def __init__(self):
        logging.info("Using dummy executor")
        self.nextEvents = [] # list of dictionnary with time/callback to call
        
    def move_aav(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("moving {w}(aav) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def observe_agv(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("{w} observe from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def move_agv(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("moving {w}(agv) from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def observe_aav(self, who, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("{w} observe from {a} to {b} in {d}".format(w=who,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def communicate_aav_agv(self, who1, who2, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(agv) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
        
    def communicate_aav_aav(self, who1, who2, a, b, cb, actionJson, **kwargs):
        dur = actionJson["dMin"]
        logging.info("Com between {w1}(aav) at {a} with {w2}(aav) at {b} in {d}".format(w1=who1,w2=who2,a=a,b=b,d=dur))
        
        currentTime = time.time()
        self.nextEvents.append( {"time":(currentTime + actionJson["dMin"]),"cb": cb, "actionJson":actionJson})
                
        
    def update(self):
        currentTime = time.time()

        for d in [d for d in self.nextEvents if d["time"] <= currentTime]:
            d["cb"]("ok")
            logging.info("calling a callback for %s" % d["actionJson"]["name"])
            
        self.nextEvents = [d for d in self.nextEvents if d["time"] > currentTime]

class MORSEActionExecutor(AbstractActionExecutor):
    def __init__(self):
        import pymorse
        self.morse = pymorse.Morse()
        logging.info("MORSE executor initialized")

    def __del__(self):
        del self.morse        

    def move_aav(self, who, a, b, cb, **kwargs):
        agent = getattr(self.morse, who)
        # pt_aav_22229_-2588
        coords = re.findall('-?\d+', b)
        logging.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        agent.waypoint.goto(int(coords[0])/100, int(coords[1])/100, 30.0, 3, 3)
        # Leave a couple of millisec to the simulator to start the action
        time.sleep(1)
        # waits until we reach the target
        while agent.waypoint.get_status() != "Arrived":
            time.sleep(5)
        cb("ok")

    
