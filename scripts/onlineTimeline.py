#!/usr/bin/env python

from __future__ import division

import logging; logger = logging.getLogger("hidden")

from plotWindow import PlotWindow

import rospy
from roshidden.msg import StnVisu
from std_msgs.msg import Empty

import functools
import itertools
import json
import time
import sys, random
import os

from PyQt4.QtCore import *
from PyQt4.QtGui import *

class OnlineTimeline(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)

        self.window_size=20
        
        self.data = {}
        self.colors = itertools.cycle(["r","g","b"])

        self.mutex = True
        
        self.beginDate = time.time()

        rospy.init_node('visu', anonymous=True)
        self.subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, self.plotResults, queue_size = 1 )

        self.subscriber = rospy.Subscriber("/hidden/start", Empty, self.start, queue_size = 1 )
        
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
        else:
            return name
    
    def plotResults(self, data): 
        if data is not None:
            if data.agent not in self.data:
                self.data[data.agent] = {"color" : next(self.colors), "index":1+len(self.data)}

            self.data[data.agent]["actions"] = []
            for m in data.actions:
                self.data[data.agent]["actions"].append({"name":m.name, "timeStart": m.timeStartLb,  "timeEnd": m.timeEndLb, "executed":m.executed, "executing":m.executing, "hierarchical":m.hierarchical})

            logger.info("Received message from %s" % data.agent)

        while not self.mutex:
            rospy.sleep(0.1)
        self.mutex = False
        
        self.axes.clear()        
        self.axes.set_autoscaley_on(False)
        
        if self.data:
            xmax = max([a["timeEnd"] for d in self.data.values() for a in d["actions"]])
        else:
            xmax = 1
        self.axes.set_xlim(0, xmax)
        
        captions = [" " for _ in range(len(self.data)+2)]
        for robot in sorted(self.data.keys()):
            posCycle = itertools.cycle([0.1,0.2,0.3,0.4,0.5])
            for action in sorted(self.data[robot]["actions"], key=lambda x: x["timeStart"]):

                if "observe" in action["name"] or action["hierarchical"]:
                    continue
                    
                if "has-communicated" in action["name"]:
                    self.axes.vlines(action["timeStart"], index-0.1, index+0.1, color="m", lw=5)
                    
                    continue

                index = self.data[robot]["index"]
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
                self.axes.axhspan(index-0.05, index+0.05, xstart/xmax, xstop/xmax, facecolor=color, edgecolor="k", lw=1, ls="solid", alpha=alpha)
                
            captions[index] = robot

        self.axes.set_yticks(range(len(captions)))
        self.axes.set_yticklabels(captions)

        self.axes.vlines(self.getCurrentTime(), 0, len(captions), color="r")
                
        self.canvas.draw()
        
        self.mutex = True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    sh = logging.StreamHandler()
    sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s %(process)d (%(filename)s:%(lineno)d): %(message)s'))
    logger.addHandler(sh)
        
    iconePath = os.path.expandvars("$ACTION_HOME/ressources/images/icone_action.png")
    if os.access(iconePath, os.R_OK):
        app.setWindowIcon(QIcon(iconePath))

    window = OnlineTimeline()
    window.show()
    
    timer = QTimer()
    timer.timeout.connect(functools.partial(window.plotResults, None))
    timer.start(1000)
    
    app.exec_()
