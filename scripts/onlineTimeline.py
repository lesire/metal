#!/usr/bin/env python

from __future__ import division

import logging; logger = logging.getLogger("hidden")

from plotWindow import PlotWindow

import rospy
from roshidden.msg import StnVisu

import itertools
import json
import sys, random

from PyQt4.QtCore import *
from PyQt4.QtGui import *

class OnlineTimeline(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=20
    
    self.data = {}
    self.colors = itertools.cycle(["r","g","b"])

    self.mutex = True

    rospy.init_node('visu', anonymous=True)
    self.subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, self.plotResults, queue_size = 1 )

    rospy.logwarn("Init done for onlineTimeline")

  def plotResults(self, data): 
    if data.agent not in self.data:
        self.data[data.agent] = {"color" : next(self.colors), "index":1+len(self.data)}

    self.data[data.agent]["actions"] = []
    for m in data.actions:
        self.data[data.agent]["actions"].append({"name":m.name, "timeStart": m.timeStartLb,  "timeEnd": m.timeEndLb, "executed":m.executed, "executing":m.executing, "hierarchical":m.hierarchical})

    rospy.logwarn("received message from %s" % data.agent)

    while not self.mutex:
      rospy.sleep(0.1)
    self.mutex = False
    
    self.axes.clear()        
    self.axes.set_autoscaley_on(False)
    
    xmax = max([a["timeEnd"] for d in self.data.values() for a in d["actions"]])
    self.axes.set_xlim(0, xmax)
    
    captions = [" " for _ in range(len(self.data)+2)]
    for robot in sorted(self.data.keys()):
        posCycle = itertools.cycle([0.1,0.2,0.3,0.4,0.5])
        for action in sorted(self.data[robot]["actions"], key=lambda x: x["timeStart"]):

            if "has-communicated" in action["name"] or "observe" in action["name"] or action["hierarchical"]:
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

            offset = next(posCycle)
            self.axes.text((xstart + xstop)/2, index + offset, action["name"],horizontalalignment='center',verticalalignment='center',fontsize=8)
            self.axes.vlines((xstart + xstop)/2, index, index + offset - 0.05)
            self.axes.axhspan(index-0.05, index+0.05, xstart/xmax, xstop/xmax, facecolor=self.data[robot]["color"], edgecolor="k", lw=1, ls="solid", alpha=alpha)
            captions[index] = robot

    self.axes.set_yticks(range(len(captions)))
    self.axes.set_yticklabels(captions)

    self.canvas.draw()
    
    self.mutex = True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineTimeline()
    window.show()
    app.exec_()
