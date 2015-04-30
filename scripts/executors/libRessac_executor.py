from .action_executor import AbstractActionExecutor

import logging; logger = logging.getLogger("hidden")

try:
    import libRessac

    # Simulation : connect to pty and automatic route launch
    TTY="/tmp/libRessac-Simu-26-to-payload.pty"
    PHASE_ROUTE=libRessac.PhaseRoute
    # Otherwise : connect to real device and wait operator for route triggering
    #TTY="/dev/ttyS0"
    #PHASE_ROUTE=libRessac.PhaseHovering

    ALT=40
    SPD=3
    DIST=6

    class libRessacExecutor(AbstractActionExecutor):
        _name = "libRessac"

        #
        # Move states :
        #
        class MoveState:
            def __init__(self, x, y):
                self.state = "initial"
                self.dst_x = x
                self.dst_y = y
                self.wp_id = -1
                self.mi_id = -1

            def isFinal(self):
                if (self.state == "final"):
                    return 1
                else:
                    return 0

        #
        # ctor
        #
        def __init__(self, agentName="ressac1", **kwargs):

            logger.info("libRessac executor : ctor")

            # agent name and id
            self.agent = agentName
            if(self.agent == "ressac1"):
                self.id = 23
            else:
                self.id = 26

            logger.info("libRessac executor : agent %s (%d)" % (self.agent, self.id))

            # list of dictionnary of pending events
            self.pendingEvents = []

            # libRessac specific init
            self.libRessacStart()

        #
        # dtor
        #
        def __del__(self):

            logger.info("libRessac executor : dtor")

            # libRessac specific termination
            self.libRessacStop()

        #
        # periodic update
        #
        def update(self):

            #logger.info("libRessac executor : update")

            for pe in self.pendingEvents:
                pe["state"] = pe["update"](pe["state"])
                if (pe["state"].isFinal()):
                    pe["cb"]("ok")
                    self.pendingEvents.remove(pe)


        #
        # move implementation
        #
        def move(self, who, a, b, cb, **kwargs):

            # check agent
            if(self.agent is not None and self.agent != who):
                logger.warning("libRessac executor : ignore action %s for %s" % (actionJson["name"], self.agent))
                pass

            # decode coords : ACTION convention ENU -> libRessac convention NED
            coords = b.split("_")[1:]
            x = int(int(coords[1])/100)
            y = int(int(coords[0])/100)

            logger.info("libRessac executor : move (%d;%d)" % (x, y))

            # append the event
            self.pendingEvents.append( {"update":self.libRessacMove, "state":self.MoveState(x, y), "cb": cb} )

        #
        # libRessac specific start
        #
        def libRessacStart(self):

            # System manager instance
            self.system_manager = libRessac.SystemManager()

            # Init proxy
            self.proxy = libRessac.ProxySystem()
            self.proxy.setSystemManager(self.system_manager)
            self.proxy.setId(self.id)
            self.system_manager.updateSystem(self.id, self.proxy, 1);

            # Link stuff
            self.tty = libRessac.SerialTty()
            self.pps = libRessac.SerialPps()
            self.log = libRessac.FileIo()

            # Init link
            self.link = libRessac.Link()
            self.link.setDataInterface(self.tty)
            self.link.setPpsInterface(self.pps)
            self.link.setLogInterface(self.log)
            self.link.setSystemManager(self.system_manager)
            self.link.setId(libRessac.SystemIdPayload)
            self.system_manager.updateLinkTx(self.id, self.link)

            # Configure tty
            self.tty.setPath(TTY)
            self.tty.setBaud(115200)
            self.tty.setData(8)
            self.tty.setParity('N')
            self.tty.setStop(1)
            self.tty.setVtime(1)
            self.tty.setVmin(0xff)
            self.tty.txEnable(1)

            # Configure pps
            self.pps.setPath(TTY)
            self.pps.setMaster(0)

            # Configure logger
            self.log.setLogger(1)
            self.log.setFileName(TTY)

            # Create a link thread
            self.link_thread = libRessac.Thread(self.link)
            self.link_thread.setPriority(80)
            self.link_thread.setName("link-tty")

            # Start thread
            self.link_thread.initialize()
            self.link_thread.startup()

            # Get uav from system manager
            self.uav = self.system_manager.getSystem(self.id)

        #
        # libRessac specific stop
        #
        def libRessacStop(self):

            # Stop thread
            self.link_thread.shutdown()
            self.link_thread.finalize()

        #
        # libRessac specific move
        #
        # initial -> ready -> ongoing -> reached -> final
        #                             -> error
        def libRessacMove(self, move):

            # get current avionic state
            state_basic = libRessac.StateBasic()
            self.uav.getStateBasic(state_basic)

            # compute distance to go to next wp
            dtg = (state_basic.x - move.dst_x)**2 + (state_basic.y - move.dst_y)**2

            if (move.state == "initial"):

                if (dtg < DIST**2) and \
                   ((state_basic.phase == libRessac.PhaseHovering) or \
                    (state_basic.phase == libRessac.PhaseHoveringOnTarget)):

                    move.state = "final"

                else:

                    logger.info("libRessac executor : exec move")

                    waypoint = libRessac.Waypoint()
                    waypoint.id = libRessac.WaypointIdPayloadMin
                    waypoint.flags = libRessac.DataFlagReady
                    waypoint.x = move.dst_x
                    waypoint.y = move.dst_y
                    waypoint.h = ALT
                    waypoint.psi = 0
                    waypoint.beta = 0
                    waypoint.speed = SPD
                    waypoint.dist = DIST
                    waypoint.phase_rt = libRessac.PhaseRoute
                    waypoint.phase_wp = libRessac.PhaseHoveringOnTarget
                    waypoint.wait = 0
                    self.uav.setWaypoint(waypoint)
                    print(waypoint)

                    route = libRessac.Route()
                    route.id = libRessac.RouteIdPayloadMin
                    route.flags = libRessac.DataFlagReady
                    route.nb_wps = 1
                    route.nb_repeat = 0
                    route.wps[0] = waypoint.id
                    self.uav.setRoute(route)
                    print(route)

                    mission = libRessac.Mission()
                    mission.id = libRessac.MissionIdPoolQuery
                    mission.flags = libRessac.DataFlagReady
                    mission.id_route = route.id
                    mission.acked = 1
                    mission.status = 0
                    self.uav.getMission(mission)
                    print(mission)

                    phase = libRessac.Phase()
                    phase.phase = PHASE_ROUTE
                    phase.cons_type = libRessac.ConsTypeMission
                    phase.cons.mission.mission_id = mission.id
                    phase.cons.mission.waypoint_id = 0
                    self.uav.setPhase(phase)
                    print(phase)

                    move.wp_id = waypoint.id
                    move.mi_id = mission.id
                    move.state = "ready"

            elif (move.state == "ready"):

                if (state_basic.phase == libRessac.PhaseRoute) and (state_basic.wp_current == move.wp_id):

                    move.state = "ongoing"

                elif ((state_basic.phase != libRessac.PhaseHovering) and \
                      (state_basic.phase != libRessac.PhaseHoveringOnTarget)):

                    move.state = "error"

            elif (move.state == "ongoing"):

                if (dtg < DIST**2) and \
                   ((state_basic.phase == libRessac.PhaseHovering) or \
                    (state_basic.phase == libRessac.PhaseHoveringOnTarget)):

                    move.state = "reached"

                elif (state_basic.phase != libRessac.PhaseHovering) and \
                     (state_basic.phase != libRessac.PhaseHoveringOnTarget) and \
                     ((state_basic.phase != libRessac.PhaseRoute) or (state_basic.wp_current != move.wp_id)) :

                    move.state = "error"

            elif (move.state == "reached") or (move.state == "error"):

                mission = libRessac.Mission()
                mission.id = move.mi_id
                mission.flags = libRessac.DataFlagFree
                self.uav.setMission(mission)

                move.state = "final"

            logger.info("libRessac executor : move state : %s" % move.state)

            return move

except ImportError:
    pass
