from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")
import re
from functools import partial
import os

class MORSEActionExecutor(AbstractActionExecutor):
    _name = "morse"

    def __init__(self, **kwargs):
        self._connected = False

        self.current_action = None
        self.cancelled_actions = set()
        logger.info("MORSE executor initialized")

    def _get_morse_host(self):
        return "localhost"

    def update(self):
        try:
            if not self._connected:
                import pymorse
                host = self._get_morse_host()
                self.morse = pymorse.Morse(host=host)
                logger.info("Connected to MORSE")
                self._connected = True
        except Exception as e:
            pass

    def _stop(self, action):
        if action["name"].split(" ")[0] == "move":
            if self.current_action is not None:
                logger.info("Cancelling the current action")
                self.current_action.cancel()

            self.cancelled_actions.add(action["name"])

    def __del__(self):
        del self.morse

    def _goto(self, x, y):
        agent = getattr(self.morse, self.agent)
        if "ressac" in self.agent:
            goto_action = agent.waypoint.goto(x, y, 30.0, 3, 2.0)
        elif "effibot" in self.agent:
            goto_action = agent.waypoint.goto(x, y, 0.0, 3, 1)
        else:
            goto_action = agent.waypoint.goto(x, y, 0.0, 3, 0.5)
        return goto_action

    def action_done(self, cb, name, evt):
        logger.info("Action done {e}. Name : {name}.".format(e=evt,name=name))
        if name in self.cancelled_actions:
            logger.info("It was canceled : do not call the callback")
            self.cancelled_actions.remove(name)
        else:
            cb("ok")
    
    def move(self, who, a, b, cb, **kwargs):
        coords = b.split("_")[1:]
        if who != self.agent:
            logger.error("Received an action that is not for me : %s %s" % (who, self.agent))
        
        logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        goto_action = self._goto(int(coords[0])/100, int(coords[1])/100)

        goto_action.add_done_callback(partial(self.action_done, cb, "move %s %s %s" %(who,a,b)))
        self.current_action = goto_action

    def observe(self, who, point, observation, cb, **kwargs):
        cb("ok")

    def communicate(self, dude, sweet, point_dude, point_sweet, cb, **kwargs):
        cb("ok")

    def track(self, x, y, cb, *args, **kwargs):
        if x is not None and y is not None:
            goto_action = self._goto(float(x),float(y))

    def has_communicated(self, first_robot, second_robot, cb, **kwargs):
        cb("ok")
