from .action_executor import DummyActionExecutor

import logging; logger = logging.getLogger("hidden")

import json
import threading
import time

try:
    import rospy
    from std_msgs.msg import Empty, String
    from metal.msg import *
    from metal.srv import *

    """
    Provides several capabilities:
      - communication with ROS
      - listen for injected alea
      - broadcast information for creating statistics
    
    For the communications :
    Uses a unique topic (with in/out) for vnet capacities.
    
    Messages are split in two : header and payload.
    The header identify the sender, the receiver, the status of the com (acked or not) and unique information (communication points)
    
    During a communicate action, will ack all the matching incomming communication.
    While the other has not received it, will also send its payload.
    
    During a communicate-meta action, periodically send the payload
    """
    class ROSActionExecutor(DummyActionExecutor):
        _name = "ros"

        def __init__(self, agentName, **kwargs):
            DummyActionExecutor.__init__(self, agentName, **kwargs)
            self.name = agentName
            
            ## Communication ##
            self._current_msg = None #if not None, assume we are in com and this is the message to send
            self._com_cb = None
            
            self._in_com_meta = False
            self._com_meta_cb = None
            
            self._com_succeded = False #true if received the message from the other side
            
            self.last_time_sent = time.time()
            self.time_gap = 0.1 # min time between resend, in seconds

            self._com_sub = rospy.Subscriber("communicate/in", Communication, self._receiveCom)
            self._com_pub = rospy.Publisher( "communicate/out", Communication, queue_size=10)

            self._com_mutex = threading.RLock()
            
            ## Injecting aleas ##
            self._alea_srv = rospy.Service("executor/alea", AleaAction, self.aleaReceived)

            ## Broadcast statistics ##
            self._stats_pub = rospy.Publisher("/hidden/stats", String, queue_size=10)

            logger.info("ROS executor initialized")
        
        ## Communications ##

        def _stop(self, action):
            with self._com_mutex:
                if "communicate-meta" in action["name"]:
                    logger.info("end of a communicate-meta action")
                    self._in_com_meta = False
                elif "communicate" in action["name"]:
                    self._current_msg = None

        def _receiveCom(self, msg):
            if msg.header.receiver != self.name: return # not for me
            
            with self._com_mutex:
                if self._current_msg is None: return # Receive a com when not in a communicate action. TODO
                
                logger.info("Received a com message")
                
                if msg.header.sender == self._current_msg.header.receiver \
                    and msg.header.receiver == self._current_msg.header.sender \
                    and msg.header.sender_position == self._current_msg.header.receiver_position \
                    and msg.header.receiver_position == self._current_msg.header.sender_position:
                    # matching header
                    logger.info("Received a com message for me")
                
                    self._com_succeded = True
                    
                    if self._in_com_meta:
                        self._com_meta_cb("ok")
                        self._com_meta_cb = None
                        self._in_com_meta = False
            
                    if msg.header.reply_expected:
                        self._send_msg()

                else:
                    logger.warning("Received a message that I'm not expecting : %s" % msg.header)
                    #ignore it, not the one I'm expecting
                    pass

        def update(self):
            DummyActionExecutor.update(self)

            if self._in_com_meta:
                self._send_msg()
                
            #TODO timeout

        def _send_msg(self):
            with self._com_mutex:
                if self._current_msg is not None and (time.time() - self.last_time_sent) > self.time_gap:
                    logger.info("Sending a com message")
                    
                    self._current_msg.header.reply_expected = not self._com_succeded
                    self._com_pub.publish(self._current_msg)
                    self.last_time_sent = time.time()

        # communicate ressac1 ressac2 pt_aav_10249_-4585 pt_aav_12245_-4585
        def communicate(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
            logger.info("Start communicate action")
            self._com_succeded = False

            self._current_msg = Communication(CommunicationId(first_robot, second_robot, first_point, second_point, True), [])
            self.com_cb = cb

        def communicate_meta(self, first_robot, second_robot, first_point, second_point, cb, **kwargs):
            with self._com_mutex:
                logger.info("Start communicate-meta action")
                self._in_com_meta = True
                self._com_meta_cb = cb
                self._send_msg()


        ## Injecting aleas ##
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
            elif msg.aleaType == "targetFound":
                logger.info("Received a alea of type target")

                pos = data.get("position", {})
                x = pos.get("x", 0)
                y = pos.get("y", 0)

                self.outQueue.put({"type":"alea", "aleaType":"targetFound", "position":{"x":x, "y":y}})

                return True
            elif msg.aleaType == "nextReport":
                logger.info("Received an alea setting the value of the next report")

                if "report" not in data:
                    logger.error("Mission the report field")
                    return False
                r = data["report"]
                self.nextReport = r

                logger.info("Set the next report to %s" % r)

                return True
            elif msg.aleaType == "robotDead":
                #forward it
                return rospy.ServiceProxy("alea", AleaAction)(msg.aleaType, msg.data)
            else:
                logger.error("Unknown alea received : %s. %s" % (msg.aleaType, data))
                return False

            return False

        ## Broadcast statistics ##
        def observe(self, who, a, b, cb, actionJson, **kwargs):
            super(ROSActionExecutor, self).observe(who, a, b, cb, actionJson, **kwargs)
            
            if "time" not in kwargs:
                logger.error("Cannot know the start time of %s" % actionJson["name"])
            t = kwargs.get("time", None)

            self._stats_pub.publish(json.dumps({"type":"observe", "by":who, "from":a, "to":b, "time":t}, sort_keys=True))

        def stopExecutor(self, time):
            super(ROSActionExecutor, self).stopExecutor(time)

            self._stats_pub.publish(json.dumps({"type":"stop", "by":self.name, "time":time}, sort_keys=True))




except ImportError:
    logger.warning("Cannot import ROS")
    pass

