import matplotlib.pyplot as plt
import numpy as np
import re
import sys

"""
Obsolete ?
Redondant with the one in simato (see actionvisu.py)
"""

# Transform input PDDL into arrays
# 0.001: (move ressac1 ptAav_16239_5397 ptAav_8252_1404) [41.264]
actions = {'ressac1': {'move': [], 'observe': [], 'communicate': [], 'communicate': [], 'has-communicated': []}
    , 'ressac2': {'move': [], 'observe': [], 'communicate': [], 'communicate': [], 'has-communicated': []}
    , 'mana': {'init':[], 'move': [], 'observe': [], 'communicate': [], 'has-communicated': []}
    , 'minnie': {'init':[], 'move': [], 'observe': [], 'communicate': [], 'has-communicated': []}}

for l in open(sys.argv[1]):
    m = re.match("^(\d*(?:.\d*)?)\s*:\s*\((.*)\)\s*\[(\d*(?:.\d*)?)\]", l)
    if m:
        tStart, actionName, dur = m.groups()

        if "communicate" in actionName:
            a,r1,r2,wp1,wp2 = actionName.split(" ")
            actions[r1][a] += [(float(tStart), float(dur))]
            actions[r2][a] += [(float(tStart), float(dur))]
        elif "init" in actionName:
            a,r1,wp1 = actionName.split(" ")
            actions[r1][a] += [(float(tStart), float(dur))]
        else:
            print actionName
            a,r1,wp1,wp2 = actionName.split(" ")
            actions[r1][a] += [(float(tStart), float(dur))]
        print(actions)

# Plot function
def timelines(y, xstart, xstop, color='b'):
    """Plot timelines at y from xstart to xstop with given color."""   
    plt.hlines(y, xstart, xstop, color, lw=4)
    #plt.vlines(xstart, y+0.03, y-0.03, color, lw=2)
    #plt.vlines(xstop, y+0.03, y-0.03, color, lw=2)

captions = ["{a}-{ac}".format(a=agent, ac=action) for agent, b in actions.items() for action in b.keys()]
y = 0
for agent, v in actions.items():
#    y += 1
    for action, ins in v.items():
        for i in ins:
            timelines(y, i[0], i[0]+i[1], 'k')
        y += 1

#Setup the plot
ax = plt.gca()
ax.set_yticks(range(len(captions)))
ax.set_yticklabels(captions)
plt.xlabel('Time')
plt.show()

