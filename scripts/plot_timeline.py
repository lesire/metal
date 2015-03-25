import matplotlib.pyplot as plt
import numpy as np
import re
import sys

# Transform input PDDL into arrays
# 0.001: (move-aav ressac1 pt_aav_16239_5397 pt_aav_8252_1404) [41.264]
actions = {'ressac1': {'move-aav': [], 'observe-aav': [], 'communicate-aav-aav': [], 'communicate-aav-agv': [], 'has-communicated': []}
    , 'ressac2': {'move-aav': [], 'observe-aav': [], 'communicate-aav-aav': [], 'communicate-aav-agv': [], 'has-communicated': []}
    , 'mana': {'move-agv': [], 'observe-agv': [], 'communicate-aav-agv': [], 'has-communicated': []}
    , 'minnie': {'move-agv': [], 'observe-agv': [], 'communicate-aav-agv': [], 'has-communicated': []}}

for l in open(sys.argv[1]):
    if l[0] == ';':
        continue
    data = re.split('[ \:\(\)\[\]]', l)
    if "communicate" in data[3]:
        actions[data[4]][data[3]] += [(float(data[0]), float(data[-2]))]
        actions[data[5]][data[3]] += [(float(data[0]), float(data[-2]))]
    else:
        actions[data[4]][data[3]] += [(float(data[0]), float(data[-2]))]
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

