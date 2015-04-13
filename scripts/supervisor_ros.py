import logging; logger = logging.getLogger("hidden")

import rospy
import json
from std_msgs.msg import Empty,String
from supervisor import Supervisor

class SupervisorRos(Supervisor):
    def __init__ (self, inQueue, outQueue, planStr, agent = None, pddlFiles=None):
        Supervisor.__init__(self, inQueue, outQueue, planStr, agent, pddlFiles)
        if agent is None:
            logging.error("Cannot repair with ROS without an agent name")
            sys.exit(1)
        self.repair_sub = rospy.Subscriber("/hidden/repair", String, self.repairCallback)
        self.repair_pub = rospy.Publisher('/hidden/repair', String, queue_size=10)

    def init(self, plan, agent):
        Supervisor.init(self, plan, agent)
        stn = open(str(self.agent) + "_STN.json", "w")
        stn.write(self.plan.stn.export())
        '''self.plan.stn.toMacroSTN()
        macro_stn = open(str(self.agent) + "_macroSTN.json", "w")
        macro_stn.write(self.plan.stn.export())'''

    def sendRepairMessage(self, msg):
        logger.info("Sending repair msg : %s" %  msg)
        self.repair_pub.publish(msg)

    def repairCallback(self, msg):
        logger.info(msg)
        logger.info(type(msg))
        logger.info(dir(msg))
        try:
            data = json.loads(msg.data)
        except TypeError:
            logger.error("Receive a repair message that is not a json string : %s" % msg.data)
            return
        logger.info("Received : %s" % data)
        
        if data["type"] == "repairRequest":
            msg = {"agent":self.agent, "type":"repairResponse", "plan":self.plan.getJsonDescription()}
            logger.info("Received a repair request")

        elif data["type"] == "repairResponse":
            if data["agent"] not in self.repairResponse:
                self.repairResponse[data["agent"]] = data["plan"]
                logger.info("Receive a repair response from %s " % data["agent"])
            else:
                logger.error("Received several response from %s. Keeping only the first one" % data["agent"])
        else:
            logger.warning("Received unsupported message of type %s : %s" % (data["type"], msg))
