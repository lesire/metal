import logging; logger = logging.getLogger("hidden")
import re
import json

try:
    from .ros_executor import ROSActionExecutor

    class HyperActionExecutor(ROSActionExecutor):
        _name = "hyper"

        def __init__(self, agentName, **kwargs):
            import yarp
            yarp.Network.init()
            self.output_port = yarp.BufferedPortBottle()
            self.output_port.open("/" + agentName + "/executor/out")
            self.input_port = yarp.BufferedPortBottle()
            self.input_port.open("/" + agentName + "/executor/in")

            self.currentAction = None
            self.isTracking = False
            self.hasSeenTarget = False

            ROSActionExecutor.__init__(self, agentName)

        def __del__(self):
            self.output_port.close()
            self.input_port.close()

        def _sendCommand(self, action):
            b = self.output_port.prepare()
            b.clear()
            b.addString(json.dumps(action))
            self.output_port.write()

        # move mana agv_pt_-726_2375 agv_pt_3323_175
        def move(self, who, a, b, cb, **kwargs):

            coords = b.split("_")[1:]
            logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
            action = {'action': 'goto', 'waypoint': {'x': int(coords[0])/100, 'y': int(coords[1])/100}}
            self._sendCommand(action)            
            logger.info("Request action " + json.dumps(action))
            self._cb = cb

        # observe mana pt_agv_22229_-2588 pt_obs_22229_-2588
        def observe(self, who, point, observation, cb, **kwargs):
            cb("ok")

        def update(self):
            ROSActionExecutor.update(self)
            try:
                b = self.input_port.read(False)
                if b is not None:
                    logger.info("Received report " + b.toString())
                    r = json.loads(b.get(0).toString())
                    r["type"] = r["report"]

                    if r["type"] == "blocked":
                        self._sendCommand(action)
                        return
                    elif "target" in r["type"]:
                        self.hasSeenTarget = True
                    self._cb(r)
            except Exception as e:
                logger.warning(e)

        def track(self, x, y, *args, **kwargs):
            self.isTracking = True
            self._cb = cb
            if not self.hasSeenTarget:
                #TODO goto_target and track ?
                logger.info("moving to target".format(w=who,a=a,b=str(coords)))
                action = {'action': 'goto_target', 'waypoint': {'x': int(x)/100, 'y': int(y)/100}}
                self._sendCommand(action)            
                logger.info("Request action " + json.dumps(action))
            else:
                logger.info("tracking")
                action = {'action': 'track'}
                self._sendCommand(action)
                logger.info("Request action " + json.dumps(action))


except ImportError:
    pass
