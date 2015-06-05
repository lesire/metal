#!/usr/bin/env python3

import tkinter as tk
import subprocess
import threading
from functools import partial
import time


###  Configuration ###

# Initial hosts
# The entries are sorted first according to their groups (alphabetically) and then alphabetically
# inside each group.

hosts = {
          "PC Mission" :                 {"addr" : "localhost",     "group":"PC Opérateur"},
          "Ressac 1 : Station Sol" :     {"addr" : "11.11.22.33",   "group":"Ressac 1"},
          "Ressac 1 : Charge utile" :    {"addr" : "11.11.22.34",   "group":"Ressac 1"},
          "Ressac 2 : Station Sol" :     {"addr" : "11.11.22.35",   "group":"Ressac 2"},
          "Ressac 2 : Charge utile" :    {"addr" : "11.11.22.36",   "group":"Ressac 2"}
        }

######################

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.pack()
        
        self.groups = sorted(set([d["group"] for d in hosts.values()]))
        self.hosts = hosts
        
        self.createWidgets()

    def createWidgets(self):
        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "Refresh"
        self.hi_there["command"] = self.update
        self.hi_there.pack(side="top")
        
        for group in self.groups:
            txt = tk.Label(self, text=group,  fg="blue")
            txt.pack(side="top")

            # sort alphabetically inside a group
            for k in sorted([k for k,d in self.hosts.items() if d["group"] == group]):
                d = self.hosts[k]
                txt = tk.Label(self, text=k,  fg="black")
                txt.pack(side="top")
                d["widget"] = txt

        self.QUIT = tk.Button(self, text="QUIT", command=root.destroy)
        self.QUIT.pack(side="bottom")

    def update(self):
        children = []
        for d in self.hosts.values():
            #call self.updateEntry(d) in different threads
            t = threading.Thread(target = partial(self.updateEntry, d))
            t.start()
            children.append(t)
        
        #Waiting for those thread blocks the GUI
        
        print("Refresh launched")

    def updateEntry(self, d):
        r = self.ping(d["addr"])
        if r:
            d["status"] = "ok"
            d["widget"]["bg"] = "green"
        else:
            d["status"] = "ko"
            d["widget"]["bg"] = "red"
        print("Done for %s" % d["addr"])

    def ping(self, addr):
        command = "ping -c 1 -t 1 %s" % addr
        try:
            c = subprocess.call(command.split(" "), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            return (c == 0)
        except subprocess.TimeoutExpired:
            print("Command %s timeouted"  % command)
            return False

root = tk.Tk()
root.title("Liveness check")
app = Application(master=root)

# Window on top
root.lift()

# Window on top, on OSX
root.call("wm", "attributes", ".", "-topmost", True)
root.after_idle(root.call, "wm", "attributes", ".", "-topmost", False)

app.mainloop()
