import logging; logger = logging.getLogger("hidden")

from .action_executor import DummyActionExecutor

import json

try:
    import rospy
    from metal.srv import AleaAction

    class DummyAleaActionExecutor(DummyActionExecutor):
        _name = "dummy-alea"
    
        def __init__(self, agentName=None, **kwargs):
            DummyActionExecutor.__init__(self, agentName=agentName)
            
            logger.info("Executor listening for alea on ros service executor/alea")
            self.alea_srv = rospy.Service("executor/alea", AleaAction, self.aleaReceived)
            
        def aleaReceived(self,msg):
                #logger.warning(msg)
                try:
                    data = json.loads(msg.data)
                    
                    if type(data) != dict:
                        logger.error("Received an ill formated AleaAction service call. Data should be a dict : %s" % data)
                        return False
                except ValueError:
                    logger.error("Received an AleaAction service call but the filed data is not json-encoded : %s " % msg.data)
                    return False
                if msg.aleaType == "delay":
                    if not "delay" in data:
                        logger.error("Received an ill formated AleaAction service call for a delay %s" % data)
                        return False

                    try:
                        d = float(data["delay"])
                    except ValueError:
                        logger.error("Cannot convert the delay field to a numeric value : %s" % data["delay"])
                        return False
                    
                    with self.eventsLock:
                        if len(self.nextEvents) == 0:
                            logger.warning("No action currently being executed")
                    
                        for event in self.nextEvents:
                            event["time"] += d
                            logger.info("Adding %s to the current action : %s" % (d, event["actionJson"]["name"]))
                    return True
                elif msg.aleaType == "target":
                    logger.info("Received a alea of type target")
                    logger.error("Not implemented yet")
                    return False
                elif msg.aleaType == "robotDead":
                    #forward it
                    return rospy.ServiceProxy("alea", AleaAction)(msg.aleaType, msg.data)
                else:
                    logger.error("Unknown alea received : %s. %s" % (msg.aleaType, data))
                    return False

                return False

except ImportError:
    logger.warning("Cannot import ROS")
    pass
