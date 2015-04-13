from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")
import re
from functools import partial

class MORSEActionExecutor(AbstractActionExecutor):
    _name = "morse"

    def __init__(self, **kwargs):
        self._connected = False
        logger.info("MORSE executor initialized")

    def update(self):
        try:
            if not self._connected:
                import pymorse
                self.morse = pymorse.Morse()
                logger.info("Connected to MORSE")
                self._connected = True
        except:
            pass

    def __del__(self):
        del self.morse

    def action_done(self, cb, evt):
        logger.debug("Action done {e}".format(e=evt))
        cb("ok")
    
    def move(self, who, a, b, cb, **kwargs):
        agent = getattr(self.morse, who)
        coords = b.split("_")[1:]
        logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        goto_action = agent.waypoint.goto(int(coords[0])/100, int(coords[1])/100, 30.0, 3, 3)
        goto_action.add_done_callback(partial(self.action_done, cb))

    def observe(self, who, point, observation, cb, **kwargs):
        cb("ok")

    def communicate(self, dude, sweet, point_dude, point_sweet, cb, **kwargs):
        cb("ok")

    def has_communicated(self, first_robot, second_robot, cb, **kwargs):
        cb("ok")
