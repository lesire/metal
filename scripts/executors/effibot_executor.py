import logging; logger = logging.getLogger("hidden")

try:
    import rospy
    from geometry_msgs.msg import Pose
    from std_msgs.msg import String
    from .ros_executor import ROSActionExecutor

    class EffibotActionExecutor(ROSActionExecutor):
        _name = "effibot"

        def __init__(self, agentName, **kwargs):
            ROSActionExecutor.__init__(self, agentName)
            self.move_publisher = rospy.Publisher("goto/goal", Pose, queue_size=10)
            self.move_subscriber = rospy.Subscriber("goto/status", String, self._move_cb)

            self.current_action = None
            self.cancelled_actions = set()

        def _stop(self, action):
            self.cancelled_actions.add(action["name"])
            ROSActionExecutor._stop(self, action)

        def _move_cb(self, data):
            if self.current_action in self.cancelled_actions:
                self.cancelled_actions.remove(self.current_action)
                return

            if "Success" in data.data:
                self.move_cb("ok")
            else:
                self.move_cb("error")

        # move effibot1 pt_agv_16239_-6582 pt_agv_22229_-2588
        def move(self, who, a, b, cb, **kwargs):
            coords = b.split("_")[1:]
            logger.info("moving {w} from {a} to {b}".format(w=who,a=a,b=str(coords)))
            p = Pose()
            p.position.x = int(coords[0])/100
            p.position.y = int(coords[1])/100
            self.move_publisher.publish(p)
            self.move_cb = cb
            self.current_action.add("move %s %s %s" % (who,a,b))

        def observe_agv(self, who, point, observation, cb, **kwargs):
            cb("ok")

except ImportError:
    logger.warning("Cannot import ROS")
    pass