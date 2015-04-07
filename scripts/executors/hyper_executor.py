from .ros_executor import ROSActionExecutor

import logging; logger = logging.getLogger("hidden")
import re

class HyperActionExecutor(ROSActionExecutor):
    def __init__(self, agentName):
        import yarp
        yarp.Network.init()
        self.output_port = yarp.BufferedPortBottle()
        self.output_port.open("/" + agentName + "/executor/out")
        self.input_port = yarp.BufferedPortBottle()
        self.input_port.open("/" + agentName + "/executor/in")
        ROSActionExecutor.__init__(self, agentName)

    def __del__(self):
        self.output_port.close()
        self.input_port.close()

    def move(self, who, a, b, cb, **kwargs):
        import json
        coords = re.findall('-?\d+', b)
        logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
        action = {'action': 'goto', 'waypoint': {'x': int(coords[0])/100, 'y': int(coords[1])/100}}
        b = self.output_port.prepare()
        b.clear()
        b.addString(json.dumps(action))
        self.output_port.write()
        logger.info("Request action " + json.dumps(action))
        self._cb = cb

    # observe-aav mana pt_agv_22229_-2588 pt_obs_22229_-2588
    def observe(self, who, point, observation, cb, **kwargs):
        cb("ok")

    def update(self):
        ROSActionExecutor.update(self)
        try:
            import json
            b = self.input_port.read(False)
            if b is not None:
                logger.info("Received report " + b.toString())
                r = json.loads(b.get(0).toString())
                self._cb(r["report"])
        except Exception as e:
            logger.warning(e)
