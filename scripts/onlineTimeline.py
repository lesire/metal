#!/usr/bin/env python

import logging; logger = logging.getLogger("hidden")

from plotWindow import PlotWindow

import rospy
from roshidden.msg import StnVisu

import itertools
import json
import sys, random

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import numpy

class OnlineTimeline(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=20
    
    self.data = {}
    self.colors = itertools.cycle(["r","g","b"])
    #self.values=numpy.zeros((self.window_size))
    #self.index=0

    rospy.init_node('visu', anonymous=True)
    self.subscriber = rospy.Subscriber("/hidden/stnvisu", StnVisu, self.plotResults, queue_size = 1 )

    rospy.logwarn("Init done")

  def plotResults(self, data): 
    if data.agent not in self.data:
        self.data[data.agent] = {"color" : next(self.colors), "index":1+len(self.data), "actions":json.loads(data.msg)}
    else:
        self.data[data.agent]["actions"] = json.loads(data.msg)

    print("received message from %s" % data.agent)

    self.axes.clear()        
    self.axes.set_autoscaley_on(False)
    
    captions = [" " for _ in range(len(self.data)+2)]
    for robot in sorted(self.data.keys()):
        #rospy.logwarn("%s" % self.data[robot]["actions"])
        posCycle = itertools.cycle([0.1,0.2,0.3,0.4,0.5])
        for action in sorted(self.data[robot]["actions"].values(), key=lambda x: x["timeStart"][0]):

            if "has-communicated" in action["name"]:
                continue

            index = self.data[robot]["index"]
            xstart = action["timeStart"][0]
            xstop  = action["timeEnd"][0]

            #rospy.logwarn("%s %s %s" % (index+0.2, (xstart + xstop)/2, action["name"]))
            offset = next(posCycle)
            self.axes.text((xstart + xstop)/2, index + offset, action["name"],horizontalalignment='center',verticalalignment='center',fontsize=8)
            self.axes.vlines((xstart + xstop)/2, index, index + offset - 0.05)
            self.axes.hlines(index, xstart, xstop, self.data[robot]["color"], lw=10, label=action["name"])
            captions[index] = robot

    self.axes.set_yticks(range(len(captions)))
    self.axes.set_yticklabels(captions)

    self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineTimeline()
    window.show()
    app.exec_()
