#!/usr/bin/env python3

import argparse
import functools
import logging; logger = logging.getLogger(__name__)
import json
import os
import re
import sys

def getDescription(name):
    try:
        import benchmarkGenerator
        for gen in benchmarkGenerator.AbstractPlanGen.__subclasses__():
            if gen._name == name:
                return gen._description
        return name
    except Exception as e:
        return name

def parseScenario(scenarioFolder, nominalFolder):
    resultsScenario = {}
    for d in sorted(os.listdir(scenarioFolder)):
        if re.match("^simu_\d+$", d):
            
            if not os.access(os.path.join(scenarioFolder, d, "results.json"), os.R_OK):
                #logger.error("Cannot find results.json. A problem occurred in this run")
                continue
            
            with open(os.path.join(scenarioFolder, d, "results.json")) as f:
                data = json.load(f)
                
            resultsScenario[d] = data
    
    resultsNominal = {}
    for d in sorted(os.listdir(nominalFolder)):
        if re.match("^simu_\d+$", d):
            with open(os.path.join(nominalFolder, d, "results.json")) as f:
                data = json.load(f)
                
            resultsNominal[d] = data
    
    name = os.path.basename(scenarioFolder).split("_")[1]
    print("*" * 166)
    print("%s : %s" % (name,getDescription(name)))
    print()
    print("Succes : %d\tTimeout : %d\tEchec : %d\tTotal : %s" % (len([r for r in resultsScenario.values() if r["success"]]), len([r for r in resultsScenario.values() if r["timeout"]]), len([r for r in resultsScenario.values() if r["error"]]), len(resultsScenario)))
    print()
    print("{:<35} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12}".format("","Min", "Max", "Moyenne","Moyenne nom","Diff.","Diff.(%)","Ecart type(σ)","Variation(%)","σ nom","Var. nom.(%)"))
    
    def mean(data):
        data = [d for d in data if d is not None]
        if len(data) == 0:
            return 0
        return float(sum(data))/len(data)
    def stddev(data):
        data = [d for d in data if d is not None]
        if len(data) == 0:
            return 0
        m = mean(data)
        return mean([(x-m)**2 for x in data])**0.5
    
    def oneValuePerRun(l, f=f):
        return [f(r) for r in l if r["success"]]

    def getRepairTimeList(l):
        result = []
        for r in l:
            if r["success"] and len(r["repair"]["lengths"]) > 0:
                result = result + r["repair"]["lengths"]
        result = list(map(lambda x:x/1000.0,result))
        return result

    values = [
              ("Durée de la mission (s)",               functools.partial(oneValuePerRun, f=lambda x:x["finishTime"])),
              ("Nombre de points observés",             functools.partial(oneValuePerRun, f=lambda x:x["obsPoints"]["nbr"])),
              ("Nombre de rendez-vous réussis",         functools.partial(oneValuePerRun, f=lambda x:x["coms"]["nbr"])),
              ("Nombre de réparations",                 functools.partial(oneValuePerRun, f=lambda x:x["repair"]["requestNbr"])),
              ("Temps passé en réparation (s)",         functools.partial(oneValuePerRun, f=lambda x:x["repair"]["totalTime"]/1000.)),
              ("Durée moyenne d'une réparation (s)",    getRepairTimeList),
              ("Nombre de messages transférés",         functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrForwarded"])),
              ("Nombre de messages part. transférés",   functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrPartial"])),
              ("Nombre de messages bloqués",            functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrFiltered"])),
              ("Nombre de messages total",              functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrForwarded"] + x["vNet"]["nbrPartial"] + x["vNet"]["nbrFiltered"])),
              ("Taille totale des messages (~ko)",      functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["bandwith"]/1000.0)),
              ("Nombre de suivi de cible",              functools.partial(oneValuePerRun, f=lambda x:x["target"]["nbrTrack"])),
              ]

    for name,getValues in values:
        scenarioValues = getValues(resultsScenario.values())
        nominalValues = getValues(resultsNominal.values())
        scenarioMean = mean(scenarioValues)
        nominalMean = mean(nominalValues)
        
        formatStr = "{:<35}|{:^12%s}|{:^12%s}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12%s}|{:^12.2f}|{:^12%s}|{:^12.2f}|{:^12%s}|" % (".2f" if len(scenarioValues) else "s",".2f" if len(scenarioValues) else "s", "s" if nominalMean == 0 else ".2f", "s" if scenarioMean == 0 else ".2f", "s" if nominalMean == 0 else ".2f")

        line = formatStr.format(name,
                             min(scenarioValues) if len(scenarioValues) else "",
                             max(scenarioValues) if len(scenarioValues) else "",
                             scenarioMean,
                             nominalMean,
                             scenarioMean - nominalMean,
                             (scenarioMean - nominalMean)*100/nominalMean if nominalMean != 0 else "",
                             stddev(scenarioValues),
                             stddev(scenarioValues)*100/scenarioMean if scenarioMean != 0 else "",
                             stddev(nominalValues),
                             stddev(nominalValues)*100/nominalMean if nominalMean != 0 else "")
        print(line)
    
    print("*" * 166)
def main(argv):
    parser = argparse.ArgumentParser(description="Parse a statistical simulation")
    parser.add_argument("outputFolder"   , type=os.path.abspath)
    parser.add_argument("--logLevel"       , type=str, default="info")
    args = parser.parse_args(argv)
    
    #Configure the logger
    numeric_level = getattr(logging, args.logLevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.logLevel)
    #logger.propagate = False
    logger.setLevel(numeric_level)

    if "output_nominal" not in os.listdir(args.outputFolder):
        logger.error("No nomial run found. Exiting")
        sys.exit(1)

    for d in os.listdir(args.outputFolder):
        if d.startswith("output_") and d != "output_nominal":
            parseScenario(os.path.join(args.outputFolder, d), os.path.join(args.outputFolder, "output_nominal"))

    logger.info("Done")
            
if __name__=="__main__":
    main(sys.argv[1:])