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
        method(*args, cb=partial(self.report, action, cb))

    def report(self, action, cb, report):
        logging.info("reporting for action {a}: {r}".format(a=action["name"],r=report))
        cb(action)

class MORSEActionExecutor(AbstractActionExecutor):
    def __init__(self):
        import pymorse
        self.morse = pymorse.Morse()
        logging.info("MORSE executor initialized")

    def __del__(self):
        del self.morse        

    def move_aav(self, who, a, b, cb):
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

    
