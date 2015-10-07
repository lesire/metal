#!/usr/bin/env python3

import argparse
import logging; logger = logging.getLogger(__name__)
import json
import os
import re
import sys

def parseScenario(scenarioFolder, nominalFolder):
    resultsScenario = {}
    for d in sorted(os.listdir(scenarioFolder)):
        if re.match("^simu_\d+$", d):
            with open(os.path.join(scenarioFolder, d, "results.json")) as f:
                data = json.load(f)
                
            resultsScenario[d] = data
    
    resultsNominal = {}
    for d in sorted(os.listdir(nominalFolder)):
        if re.match("^simu_\d+$", d):
            with open(os.path.join(nominalFolder, d, "results.json")) as f:
                data = json.load(f)
                
            resultsNominal[d] = data
    
    print("*" * 101)
    print("\t%s" % os.path.basename(scenarioFolder).split("_")[1])
    print()
    print("Succes : %d\tTimeout : %d\tEchec : %d" % (len([r for r in resultsScenario.values() if r["success"]]), len([r for r in resultsScenario.values() if r["timeout"]]), len([r for r in resultsScenario.values() if r["error"]])))
    print()
    print("{:<35} {:^12} {:^12} {:^12} {:^12} {:^12}".format("","Moyenne","Moyenne nom","Variation","Ecart type","Ecart type nom"))
    
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
              ("Nombre de messages tranférés", lambda x:x["vNet"]["nbrForwarded"]),
              ("Nombre de messages part. tranférés", lambda x:x["vNet"]["nbrPartial"]),
              ("Nombre de messages bloqués", lambda x:x["vNet"]["nbrFiltered"]),
              ("Taille totale des messages (~ko)", lambda x:x["vNet"]["bandwith"]/1000.0),
              ("Nombre de suivi de cible", lambda x:x["target"]["nbrTrack"]),
              ]
    

    
    for name,getValue in values:
        scenarioValues = [getValue(r) for r in resultsScenario.values() if r["success"]]
        nominalValues  = [getValue(r) for r in resultsNominal.values() if r["success"]]
        print("{:<35}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|{:^12.2f}|".format(name, mean(scenarioValues), mean(nominalValues), mean(scenarioValues) - mean(nominalValues), stddev(scenarioValues), stddev(nominalValues)))
    
    print("*" * 101)
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