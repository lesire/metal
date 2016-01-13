import rosbag
import sys

bag = rosbag.Bag("/home/pbechon/action/action_ros_ws/src/metal/data/stats_output/action-VI-2015-10-12-full/output_deadRobotIsolatedRobot/simu_10/stats.bag")

for _,msg,_ in bag.read_messages(topics="/ressac1/hidden/mastnUpdate/in"):
	if 'communicate-meta effibot1 effibot3 effipt_11949_-4580_0 effipt_12129_-4745_0' in msg.droppedComs:
		print("Executed nodes :")
		for tp in msg.executedNodes:
			if "effibot1" in tp.tpName and "effibot3" in tp.tpName:
				print(tp)
		#print(msg)
		sys.exit(0)



"""
import plan
import json

#with open("/home/pbechon/action/action_ros_ws/src/metal/data/stats_output/action-VI-2015-10-07-new/output_deadRobot/simu_0/plan_broken_2015_10_07_19_27_30_ressac1_init.plan") as f:
with open("/tmp/plan.json") as f:
	planStr = " ".join(f.readlines())


planJson = json.loads(planStr)

comMetaName = "communicate-meta effibot1 ressac2 effipt_12305_2615_0 ressac2pt_12245_1404_0"

#Remove the ub of the droped coms
tps = []
robot1,robot2 = comMetaName.split(" ")[1:3]
print(robot1,robot2)
for a in planJson["actions"].values():
	if "communicate" in a["name"] and robot1 in a["name"]:
		print(a["name"])
		print(a["name"].startswith("communicate %s %s" % (robot1,robot2)))
		print(a["name"].startswith("communicate %s %s" % (robot2,robot1)))
	if a["name"].startswith("communicate %s %s" % (robot1,robot2)) or\
	   a["name"].startswith("communicate %s %s" % (robot2,robot1)):
		tps.append(a["startTp"])
		tps.append(a["endTp"])

	if a["endTp"] == 23 or a["startTp"] == 23:
		print(a)

print(tps)

for i,tl in reversed(list(enumerate(planJson["temporal-links"]))):
	if "ub" in tl:
		print(tl)
	if "ub" in tl and tl["endTp"] in tps:
		pass#del planJson["temporal-links"][i]
		
p = plan.Plan(json.dumps(planJson), "mana")
"""
