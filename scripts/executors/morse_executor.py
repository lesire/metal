from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")
import re
from functools import partial

class MORSEActionExecutor(AbstractActionExecutor):
    def __init__(self):
        import pymorse
        self.morse = pymorse.Morse()
        logger.info("MORSE executor initialized")

    def __del__(self):
        del self.morse

    def action_done(self, cb, evt):
        logger.debug("Action done {e}".format(e=evt))
        cb("ok")
    
    # move-aav ressac2 pt_aav_16239_-6582 pt_aav_22229_-2588
    def move_aav(self, who, a, b, cb, **kwargs):
        agent = getattr(self.morse, who)
        coords = re.findall('-?\d+', b)
        logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        goto_action = agent.waypoint.goto(int(coords[0])/100, int(coords[1])/100, 30.0, 3, 3)
        goto_action.add_done_callback(partial(self.action_done, cb))

    def move_agv(self, who, a, b, cb, **kwargs):
        agent = getattr(self.morse, who)
        coords = re.findall('-?\d+', b)
        logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        goto_action = agent.waypoint.goto(int(coords[0])/100, int(coords[1])/100, 0.0, 1, 1)
        goto_action.add_done_callback(partial(self.action_done, cb))

    # observe-aav ressac2 pt_aav_22229_-2588 pt_obs_22229_-2588
    def observe_agv(self, who, point, observation, cb, **kwargs):
        cb("ok")

    def observe_aav(self, who, point, observation, cb, **kwargs):
        cb("ok")

    # communicate-aav-aav ressac1 ressac2 pt_aav_10249_-4585 pt_aav_12245_-4585
    def communicate_aav_aav(self, dude, sweet, point_dude, point_sweet, cb, **kwargs):
        cb("ok")
