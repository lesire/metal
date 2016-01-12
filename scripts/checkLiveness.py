#!/usr/bin/env python3

"""
Ping a set of host to check for liveness and connectivy
"""

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
          "PC Mission" :                 {"addr" : "134.212.28.32",    "group":"PC Opérateur"},
          "PC Mission Fixe" :            {"addr" : "134.212.241.95",    "group":"PC Opérateur"},
          "Ressac 2 : Station Sol" :     {"addr" : "134.212.244.34",   "group":"Ressac 2"},
          "Ressac 1 : Charge utile" :    {"addr" : "134.212.244.176",  "group":"Ressac 1"},
          "Ressac 1 : Station Sol" :     {"addr" : "134.212.244.35",   "group":"Ressac 1"},
          "Ressac 2 : Charge utile" :    {"addr" : "134.212.244.178",  "group":"Ressac 2"},
          "Minode" :                     {"addr" : "134.212.244.36",   "group":"Ressac"},
          "Mana" :                       {"addr" : "140.93.16.55",     "group":"LAAS"},
          "Momo" :                       {"addr" : "140.93.16.72",     "group":"LAAS"},
          "Minnie" :                     {"addr" : "140.93.16.57",     "group":"LAAS"},
          "Passerelle ONERA-LAAS" :      {"addr" : "134.212.90.100",   "group":"0 - Réseaux"},
          "Access point WiFi 1" :        {"addr" : "134.212.244.109",  "group":"0 - Réseaux"},
          "Access point WiFi 2" :        {"addr" : "134.212.244.110",  "group":"0 - Réseaux"},
          "Effibot 1" :                  {"addr" : "134.212.244.71",  "group":"Effibot"},
          "Effibot 2" :                  {"addr" : "134.212.244.72",  "group":"Effibot"},
          "Effibot 3" :                  {"addr" : "134.212.244.73",  "group":"Effibot"},
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
        print("Done %s for %s" % (d["status"], d["addr"]))

    def ping(self, addr):
        command = "ping -c 1 -W 1 %s" % addr
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
