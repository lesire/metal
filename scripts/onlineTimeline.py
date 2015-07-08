#!/usr/bin/env python3

from __future__ import division

import logging; logger = logging.getLogger("hidden")

from plotWindow import PlotWindow

import rospy
from metal.msg import StnVisu
from std_msgs.msg import Empty

import argparse
import functools
import itertools
import json
import time
import sys, random
import os

from PyQt4.QtCore import *
from PyQt4.QtGui import *

class OnlineTimeline(PlotWindow):
    def __init__(self, missionFile = None):
        PlotWindow.__init__(self)

        self.window_size=20
        
        self.data = {}
        self.colors = itertools.cycle(["r","g","b"])
        
        self.beginDate = -1

        rospy.init_node('visu', anonymous=True)
        self.subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, self.getInput, queue_size = 1 )

        self.subscriber = rospy.Subscriber("/hidden/start", Empty, self.start, queue_size = 1 )
        
        if missionFile is not None:
            with open(missionFile) as f:
                self.mission = json.load(f)
        else:
            self.mission = None
        
        logger.info("Init done for onlineTimeline")

    def start(self, m):
        self.beginDate = time.time()

    def getCurrentTime(self):
        if self.beginDate < 0:
          logger.error("getCurrentTime called before initialisation")

        return int(round(1000 * (time.time() - self.beginDate)))
    
    def getPrintName(self, name):
        if "move" in name:
            try:
                locs = []
                for l in name.split(" ")[2:]:
                    pt = []
                    for p in l.split("_")[1:3]:
                        pt.append(str(int(float(p)/100)))
                    locs.append("_".join(pt))
                #rospy.logwarn(locs)
                return "->".join(locs)
            except ValueError:
                return name
        elif "communicate" in name:
            return " ".join(name.split(" ")[1:3])
        elif "init " in name:
            return "init"
        else:
            return name

    def getInput(self, data):
        if data is not None:
            if data.agent not in self.data:
                if self.mission is not None and data.agent in self.mission["agents"]:
                    c = self.mission["agents"][data.agent]["color"]
                else:
                    c = next(self.colors)

                self.data[data.agent] = {"color" : c, "index":1+len(self.data)}
                
                #sort robot alphabetically
                for i,r in enumerate(sorted(self.data.keys(), reverse=True)):
                    self.data[r]["index"] = i+1

            self.data[data.agent]["actions"] = []
            for m in data.actions:
                self.data[data.agent]["actions"].append({"name":m.name, "timeStart": m.timeStartLb/1000,  "timeEnd": m.timeEndLb/1000, "executed":m.executed, "executing":m.executing, "hierarchical":m.hierarchical})
                
            self.data[data.agent]["state"] = data.state

            #logger.info("Received message from %s" % data.agent)

    def plotResults(self):
        self.axes.clear()        
        self.axes.set_autoscaley_on(False)
        
        actions = [a["timeEnd"] for d in self.data.values() for a in d["actions"]]
        if actions:
            xmax = max(actions)
        else:
            xmax = 1
        self.axes.set_xlim(0, xmax)
        
        captions = [" " for _ in range(len(self.data)+2)]
        for robot in sorted(self.data.keys()):
            
            index = self.data[robot]["index"]
            posCycle = itertools.cycle([0.15,0.3,0.45,0.6,0.75])
            for action in sorted(self.data[robot]["actions"], key=lambda x: x["timeStart"]):

                if "observe" in action["name"] or action["hierarchical"]:
                    continue
                    
                if "has-communicated" in action["name"]:
                    self.axes.vlines(action["timeStart"], index-0.1, index+0.1, color="m", lw=5)
                    
                    continue

                xstart = action["timeStart"]
                xstop  = action["timeEnd"]
                if action["executed"]:
                  alpha = 0.6
                elif action["executing"]:
                  alpha = 1
                else:
                  alpha = 0.2

                color = self.data[robot]["color"]

                offset = next(posCycle)
                self.axes.text((xstart + xstop)/2, index + offset, self.getPrintName(action["name"]),horizontalalignment='center',verticalalignment='center',fontsize=8)
                self.axes.vlines((xstart + xstop)/2, index, index + offset - 0.05)
                
                d = {}
                if "communicate" in action["name"]:
                    d["hatch"] = 'x'
                    d["edgecolor"] = color
                    d["facecolor"] = "w"
                else:
                    d["edgecolor"] = "k"
                    d["facecolor"] = color

                self.axes.axhspan(index-0.05, index+0.05, xstart/xmax, xstop/xmax, lw=1, ls="solid", alpha=alpha, **d)
                
            if index is not None:
                captions[index] = robot + "\n" + self.data[robot]["state"]

        self.axes.set_yticks(range(len(captions)))
        self.axes.set_yticklabels(captions)

        if self.beginDate > 0:
            self.axes.vlines(self.getCurrentTime()/1000, 0, len(captions), color="r")
                
        self.canvas.draw()
        
        self.mutex = True

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Create a visualisation for PDDL plans')
    parser.add_argument('--missionFile', type=str, default=None)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    app = QApplication(sys.argv)
    
    sh = logging.StreamHandler()
    sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s %(process)d (%(filename)s:%(lineno)d): %(message)s'))
    logger.addHandler(sh)
        
    iconePath = os.path.expandvars("$ACTION_HOME/ressources/images/icone_action.png")
    if os.access(iconePath, os.R_OK):
        app.setWindowIcon(QIcon(iconePath))

    window = OnlineTimeline(args.missionFile)
    window.show()

    if "raise_" in dir(window):
        window.raise_()

    timer = QTimer()
    timer.timeout.connect(window.plotResults)
    timer.start(1000)
    
    app.exec_()
