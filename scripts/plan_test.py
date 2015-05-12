import random
import unittest

import plan
import json

from copy import copy,deepcopy

class TestSequenceFunctions(unittest.TestCase):

    def setUp(self):
        self.seq = list(range(10))

    def test_shuffle(self):
        # make sure the shuffled sequence does not lose any elements
        random.shuffle(self.seq)
        self.seq.sort()
        self.assertEqual(self.seq, list(range(10)))

        # should raise an exception for an immutable sequence
        self.assertRaises(TypeError, random.shuffle, (1,2,3))

    def test_choice(self):
        element = random.choice(self.seq)
        self.assertTrue(element in self.seq)

    def test_sample(self):
        with self.assertRaises(ValueError):
            random.sample(self.seq, 20)
        for element in random.sample(self.seq, 5):
            self.assertTrue(element in self.seq)

class PlanTest(unittest.TestCase):

    def setUp(self):
        p = {}
        
        p["actions"] = {"0" : {"name":"dummy init", "dMax": 0.0, "dMin": 0.0, "endTp": 0,"startTp": 0},
                        "1" : {"name":"dummy end", "dMax": 0.0, "dMin": 0.0, "endTp": 1,"startTp": 1},
                        }
        p["causal-links"] = []
        p["temporal-links"] = []
        p["absolute-time"] = []
    
        self.emptyPlan = plan.Plan(copy(json.dumps(p)))
        
        p["actions"]["2"] = {"name":"observe minnie agvpt_12305_2615_0 ptobs_12070_2605","startTp":2,"endTp":3,"dMin":-1.0,"dMax":1.0,"agent":"mana"}
        p["actions"]["3"] = {"name":"observe minnie agvpt_12305_2615_0 ptobs_22070_2605","startTp":4,"endTp":5,"dMin":-1.0,"dMax":1.0,"agent":"minnie"}
        
        p["causal-links"].append({"startAction":"2","endAction":"1","startTp":3,"endTp":1,"startTs":2,"endTs":3,"lit":"explored ptobs_12070_2605"})
        p["causal-links"].append({"startAction":"3","endAction":"1","startTp":5,"endTp":1,"startTs":2,"endTs":3,"lit":"explored ptobs_22070_2605"})
        
        self.plan = plan.Plan(copy(json.dumps(p)))

    def helper_checkJsonPlan(self, data):
        self.assertEqual(type(data), dict)
        self.assertIn("actions", data)
        self.assertIn("causal-links", data)
        self.assertIn("temporal-links", data)
        self.assertIn("absolute-time", data)
        
        #check for dummy init
        self.assertIn("0", data["actions"])
        self.assertEqual("dummy init", data["actions"]["0"]["name"])
        self.assertEqual(0, data["actions"]["0"]["startTp"])
        self.assertEqual(0, data["actions"]["0"]["endTp"])
        self.assertNotIn("agent", data["actions"])
        
        #check for dummy end
        self.assertIn("1", data["actions"])
        self.assertEqual("dummy end", data["actions"]["1"]["name"])
        self.assertEqual(1, data["actions"]["1"]["startTp"])
        self.assertEqual(1, data["actions"]["1"]["endTp"])
        self.assertNotIn("agent", data["actions"])
        
        for k,a in data["actions"].items():
            if k in ["0", "1"]:
                continue
            self.assertIn("dMin", a)
            #self.assertIn("dMax", a)
            self.assertIn("startTp", a)
            self.assertIn("endTp", a)
            
        for cl in data["causal-links"]:
            self.assertIn("startTp", cl)
            self.assertIn("endTp", cl)
            self.assertIn("lit", cl)
            self.assertIn("startTs", cl)
            self.assertIn("endTs", cl)
            self.assertIn("startAction", cl)
            self.assertIn("endAction", cl)
            
            self.assertIn(cl["startAction"], data["actions"])
            self.assertIn(cl["endAction"], data["actions"])
        
            self.assertTrue(cl["startTp"] == data["actions"][cl["startAction"]]["startTp"] or
                            cl["startTp"] == data["actions"][cl["startAction"]]["endTp"])

            self.assertTrue(cl["endTp"] == data["actions"][cl["endAction"]]["startTp"] or
                            cl["endTp"] == data["actions"][cl["endAction"]]["endTp"])
        
        tps = [a["startTp"] for a in data["actions"].values()]
        tps += [a["endTp"] for a in data["actions"].values()]
        
        for tl in data["temporal-links"]:
            self.assertIn("startTp", cl)
            self.assertIn("endTp", cl)
        
            self.assertIn(tl["startTp"], tps)
            self.assertIn(tl["endTp"], tps)
        
        for t,v in data["absolute-time"]:
            self.assertIn(t, tps)
        
        pass

    def test_basic(self):
        self.assertEqual(len(self.emptyPlan.actions), 2)
        self.assertEqual(len(self.plan.actions), 4)
    
    def test_exportJson(self):
        self.helper_checkJsonPlan(self.emptyPlan.getJsonDescription())
        self.helper_checkJsonPlan(self.plan.getJsonDescription())

    def test_localPlan(self):
        p = self.emptyPlan.getLocalJsonPlan("mana")
        self.assertEqual(len(p["actions"]), 2)

        p = self.plan.getLocalJsonPlan("mana")
        self.assertEqual(len(p["actions"]), 3)

        p = self.plan.getLocalJsonPlan("minnie")
        self.assertEqual(len(p["actions"]), 3)

        p = self.plan.getLocalJsonPlan("ressac1")
        self.assertEqual(len(p["actions"]), 2)

    def test_mergeLocalPlan(self):
        p1 = self.plan.getLocalJsonPlan("mana")
        p2 = self.plan.getLocalJsonPlan("minnie")

        p = plan.Plan.mergeJsonPlans({"mana" : p1, "minnie":p2})

        self.assertEqual(len(p["actions"]), 4)
        self.helper_checkJsonPlan(p)

    def helper_mergeLocalPlanFull(self, planFile):
        with open(planFile) as f:
            l = " ".join(f.readlines())

        originalPlan = plan.Plan(l)
        originalP = originalPlan.getJsonDescription()
        self.helper_checkJsonPlan(originalP)

        d = {}
        agents = set([a["agent"] for a in originalP["actions"].values() if "agent" in a])
        
        print(agents)
        
        for agent in agents:
            d[agent] = originalPlan.getLocalJsonPlan(agent)
            self.helper_checkJsonPlan(d[agent])

        p = plan.Plan.mergeJsonPlans(d)
        self.helper_checkJsonPlan(p)

        self.assertEqual(len(p["actions"]), len(originalP["actions"]))

    def test_fullMerge1(self):
        self.helper_mergeLocalPlanFull("/Users/patrick/Documents/workspace_planner/ressources/missions/action-V-mission/hipop-files/action-V-mission.plan")


if __name__ == '__main__':
    unittest.main()