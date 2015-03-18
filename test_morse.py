from morse.builder import *

class Ressac(RMax):
    def __init__(self, x=0, y=0, z=30):
        RMax.__init__(self)
        gps = GPS()
        self.append(gps)
        waypoint = Waypoint()
        waypoint.properties(FreeZ=True, Speed=3.0)
        self.append(waypoint)
        gps.add_interface('socket')
        waypoint.add_interface('socket')
        self.translate(x, y, z)

class AGV(ATRV):
    def __init__(self, x=0, y=0):
        ATRV.__init__(self)
        self.properties(GroundRobot=True)
        gps = GPS()
        self.append(gps)
        waypoint = Waypoint()
        waypoint.properties(Speed=1.0)
        self.append(waypoint)
        gps.add_interface('socket')
        waypoint.add_interface('socket')
        self.translate(x, y, 1)
ressac1 = Ressac(162, 53)
ressac2 = Ressac(222, -6)
mana = AGV(170, 65)
minnie = AGV(170, 65)

# Scene
env = Environment("caylus50.blend")
env.set_camera_clip(clip_end=1000)
env.aim_camera([0.4, 0, -0.7854])
env.place_camera([138, -150, 100])
env.create()
