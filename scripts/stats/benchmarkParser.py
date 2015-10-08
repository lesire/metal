#!/usr/bin/env python3

import argparse
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
    print("*" * 140)
    print("%s : %s" % (name,getDescription(name)))
    print()
    print("Succes : %d\tTimeout : %d\tEchec : %d\tTotal : %s" % (len([r for r in resultsScenario.values() if r["success"]]), len([r for r in resultsScenario.values() if r["timeout"]]), len([r for r in resultsScenario.values() if r["error"]]), len(resultsScenario)))
    print()
    print("{:<35} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12} {:^12}".format("","Moyenne","Moyenne nom","Diff.","Diff.(%)","Ecart type(σ)","Variation(%)","σ nom","Var. nom.(%)"))
    
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
    
    values = [
              ("Durée de la mission (s)", lambda x:x["finishTime"]),
              ("Nombre de points observés", lambda x:x["obsPoints"]["nbr"]),
              ("Nombre de rendez-vous réussis", lambda x:x["coms"]["nbr"]),
              ("Nombre de réparations", lambda x:x["repair"]["requestNbr"]),
              ("Temps passé en réparation (s)", lambda x:x["repair"]["totalTime"]/1000.),
              ("Durée moyenne d'une réparation (s)", lambda x: mean(x["repair"]["lengths"])/1000.0 if x["repair"]["lengths"] else None),
              ("Nombre de messages transférés", lambda x:x["vNet"]["nbrForwarded"]),
              ("Nombre de messages part. transférés", lambda x:x["vNet"]["nbrPartial"]),
              ("Nombre de messages bloqués", lambda x:x["vNet"]["nbrFiltered"]),
              ("Nombre de messages total", lambda x:x["vNet"]["nbrForwarded"] + x["vNet"]["nbrPartial"] + x["vNet"]["nbrFiltered"]),
              ("Taille totale des messages (~ko)", lambda x:x["vNet"]["bandwith"]/1000.0),
              ("Nombre de suivi de cible", lambda x:x["target"]["nbrTrack"]),
              ]
    

    
    for name,getValue in values:
        scenarioValues = [getValue(r) for r in resultsScenario.values() if r["success"]]
        nominalValues  = [getValue(r) for r in resultsNominal.values() if r["success"]]
        
        if mean(nominalValues) == 0:
            print("{:<35}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12s}|{:^12.2f}|{:^12s}|{:^12.2f}|{:^12s}|".format(      name, mean(scenarioValues), mean(nominalValues), mean(scenarioValues) - mean(nominalValues), " "                                                                 , stddev(scenarioValues), " "                                            , stddev(nominalValues), ""                                             ))
        else:
            print("{:<35}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|".format(name, mean(scenarioValues), mean(nominalValues), mean(scenarioValues) - mean(nominalValues), (mean(scenarioValues) - mean(nominalValues))*100/mean(nominalValues), stddev(scenarioValues), stddev(scenarioValues)*100/mean(scenarioValues), stddev(nominalValues), stddev(nominalValues)*100/mean(nominalValues)))

    
    print("*" * 140)
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