from .action_executor import DummyActionExecutor

import logging; logger = logging.getLogger("hidden")

import libRessac

ID=26

# En simu
#TTY="/tmp/libRessac-Simu-26-to-payload.pty"
# Sur la cible
TTY="/dev/ttyS0"

class libRessacExecutor(DummyActionExecutor):
    _name = "libRessac"

    def __init__(self, agentName="ressac1", **kwargs):
        DummyActionExecutor.__init__(self, agentName)

        logger.info("libRessac executor : ctor")

        self.agent = agentName
        if(self.agent == "ressac1"):
            self.id = 23
        else:
            self.id = 26
        logger.info("libRessac executor : agent %s (%d)" % (self.agent, self.id))

        self.pendingEvents = [] # list of dictionnary of pending events

        self.libRessacStart()

    def __del__(self):
        logger.info("libRessac executor : dtor")
        self.libRessacStop()

    def update(self):
        DummyActionExecutor.update(self)
        #logger.info("libRessac executor : update")
        for pe in self.pendingEvents:
            x = pe["x"]
            y = pe["y"]
            wp_id = pe["wp_id"]
            res = self.libRessacGotoIsDone(x, y, wp_id)
            if(res != "pending"):
                logger.info("libRessac executor : goto (%d;%d) %s" % (x, y, res))
                pe["cb"]("ok")
                self.pendingEvents.remove(pe)

        
    def move(self, who, a, b, cb, **kwargs):

        # check agent
        if(self.agent is not None and self.agent != who):
            logger.warning("libRessac executor : ignore action %s for %s" % (actionJson["name"], self.agent))
            pass

        # decode coords
        coords = b.split("_")[1:]
        x = int(coords[0])
        y = int(coords[1])
        logger.info("libRessac executor : goto (%d;%d)" % (x, y))

        # start the action
        wp_id = self.libRessacGoto(x, y)
        self.pendingEvents.append( {"x":x, "y":y, "wp_id":wp_id, "cb": cb} )


    def libRessacStart(self):

        # System manager instance
        self.system_manager = libRessac.SystemManager()
        
        # Init proxy
        self.proxy = libRessac.ProxySystem()
        self.proxy.setSystemManager(self.system_manager)
        self.proxy.setId(ID)
        self.system_manager.updateSystem(ID, self.proxy, 1);
        
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
        self.system_manager.updateLinkTx(ID, self.link)
         
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
        self.uav = self.system_manager.getSystem(ID)

    def libRessacStop(self):

        # Stop thread
        self.link_thread.shutdown()
        self.link_thread.finalize()

    def libRessacGoto(self, x, y):

        waypoint = libRessac.Waypoint()
        waypoint.id = libRessac.WaypointIdPayloadMin
        waypoint.flags = libRessac.DataFlagReady
        waypoint.x = x
        waypoint.y = y
        waypoint.h = 40
        waypoint.psi = 0
        waypoint.beta = 0
        waypoint.speed = 3
        waypoint.dist = 6
        waypoint.phase_rt = libRessac.PhaseRoute
        waypoint.phase_wp = libRessac.PhaseHoveringOnTarget
        waypoint.wait = 0
        self.uav.setWaypoint(waypoint)

        route = libRessac.Route()
        route.id = libRessac.RouteIdPayloadMin
        route.flags = libRessac.DataFlagReady
        route.nb_wps = 1
        route.nb_repeat = 0
        route.wps[0] = 6
        self.uav.setRoute(route)
        
        mission = libRessac.Mission()
        mission.id = libRessac.MissionIdPayloadMin
        mission.flags = libRessac.DataFlagReady
        mission.id_route = 2
        #mission.acked = 0
        mission.acked = 1
        mission.status = 0
        self.uav.setMission(mission)
        
        phase = libRessac.Phase()
        phase.phase = libRessac.PhaseRoute
        phase.cons_type = libRessac.ConsTypeMission
        phase.cons.mission.mission_id = libRessac.MissionIdPayloadMin
        phase.cons.mission.waypoint_id = 0
        self.uav.setPhase(phase)

        while True:
            state_basic=libRessac.StateBasic()
            self.uav.getStateBasic(state_basic)
            if (state_basic.phase == libRessac.PhaseRoute):
                break

        return waypoint.id
        
    def libRessacGotoIsDone(self, x, y, wp_id):

        state_basic=libRessac.StateBasic()
        self.uav.getStateBasic(state_basic)
        
        if(state_basic.phase == libRessac.PhaseHovering):
            return "interrupted"
        elif (state_basic.phase == libRessac.PhaseRoute) and (state_basic.wp_current == wp_id):
            return "pending"
        elif (state_basic.phase == libRessac.PhaseHoveringOnTarget):
            return "ok"
        else:
            return "unexpected"

