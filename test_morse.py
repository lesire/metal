from morse.builder import *

class Ressac(RMax):
    def __init__(self, x=0, y=0, z=30):
        RMax.__init__(self)
        gps = GPS()
        self.append(gps)
        waypoint = Waypoint()
        waypoint.properties(FreeZ=True, Speed=3)
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
        waypoint.properties(Speed=1.5)
        self.append(waypoint)
        gps.add_interface('socket')
        waypoint.add_interface('socket')
        self.translate(x, y, 1)
ressac1 = Ressac(102, -45)
ressac2 = Ressac(122, -45)
mana = AGV(77, 11)
minnie = AGV(233, -17)

# Scene
env = Environment("caylusVillage-IGN-v4-forTheBuilder.blend")
env.set_camera_clip(clip_end=1000)
env.aim_camera([0.4, 0, -0.7854])
env.place_camera([138, -150, 100])
env.create()
