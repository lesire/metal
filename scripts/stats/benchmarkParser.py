#!/usr/bin/env python3

import argparse
import functools
import logging; logger = logging.getLogger(__name__)
import json
import os
import re
import sys

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import matplotlib.pyplot as plt
except:
    pass

def getDescription(name):
    try:
        import benchmarkGenerator
        for gen in benchmarkGenerator.AbstractPlanGen.__subclasses__():
            if gen._name == name:
                return gen._descriptionLatexFr
        return name
    except Exception as e:
        return name

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

def oneValuePerRun(l, f):
    return [f(r) for r in l if r["success"]]

def getRepairTimeList(l):
    result = []
    for r in l:
        if r["success"] and len(r["repair"]["lengths"]) > 0:
            result = result + r["repair"]["lengths"]
    result = list(map(lambda x:x/1000.0,result))
    return result

#Each columns is defined by its name and its function.
#The function is given two lists : the values for the current setting and the values for the nominal case
columns = [
            {"name":"Min.",            "func":lambda s,n: min(s) if len(s) else ""},
            {"name":"Max.",            "func":lambda s,n: max(s) if len(s) else ""},
            {"name":"Moyenne",         "func":lambda s,n: mean(s)},
            {"name":"Moyenne nom.",    "func":lambda s,n: mean(n), "latexFont":"\\itshape "},
            {"name":"Différence",           "func":lambda s,n: (mean(s) - mean(n))},
            {"name":"Différence (%)",        "func":lambda s,n: (mean(s) - mean(n))*100/(mean(n)) if mean(n) != 0 else ""},
            {"name":"Écart-type σ",   "func":lambda s,n: stddev(s)},
            {"name":"σ (%)",    "func":lambda s,n: stddev(s)*100/mean(s) if mean(s) != 0 else ""},
            {"name":"σ nom.",           "func":lambda s,n: stddev(n), "latexFont":"\\itshape "},
            {"name":"σ nom. (%)",    "func":lambda s,n: stddev(n)*100/mean(n) if mean(n) != 0 else "", "latexFont":"\\itshape "},
          ]

values = [
          ("Durée de la mission (s)",               functools.partial(oneValuePerRun, f=lambda x:x["finishTime"])),
          ("Nombre de points observés",             functools.partial(oneValuePerRun, f=lambda x:x["obsPoints"]["nbr"])),
          ("Nombre de rendez-vous réussis",         functools.partial(oneValuePerRun, f=lambda x:x["coms"]["nbr"])),
          ("Nombre de suivis de cible",              functools.partial(oneValuePerRun, f=lambda x:x["target"]["nbrTrack"])),
          ("Nombre de réparations",                 functools.partial(oneValuePerRun, f=lambda x:x["repair"]["requestNbr"])),
          ("Temps passé en réparation (s)",         functools.partial(oneValuePerRun, f=lambda x:x["repair"]["totalTime"]/1000.)),
          ("Durée moyenne d'une réparation (s)",    getRepairTimeList),
          ("Nombre de messages transférés",         functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrForwarded"])),
          ("Nombre de messages partiellement transférés",   functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrPartial"])),
          ("Nombre de messages bloqués",            functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrFiltered"])),
          ("Nombre total de messages",              functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["nbrForwarded"] + x["vNet"]["nbrPartial"] + x["vNet"]["nbrFiltered"])),
          ("Taille totale des messages (~ko)",      functools.partial(oneValuePerRun, f=lambda x:x["vNet"]["bandwith"]/1000.0)),
          ]

#ordering for the scenarii
namesOrdered = ["deadRobot", "deadRobotIsolatedRobot", "targetFound", "targetFoundIsolatedRobot", "doubleTargetFound", "simpleDelay", "simpleDelayIsolatedRobot", "complex"]

#Returns a table of string to be printed
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

    header = []

    name = os.path.basename(scenarioFolder).split("_")[1]
    header.append("%s : %s" % (name,getDescription(name)))
    header.append("")
    header.append("Succès : %d\tTimeout : %d\tÉchec : %d\tTotal : %s" % (len([r for r in resultsScenario.values() if r["success"]]), len([r for r in resultsScenario.values() if r["timeout"]]), len([r for r in resultsScenario.values() if r["error"]]), len(resultsScenario)))


    results = [["" for _ in range(len(columns)+1)] for _ in range(len(values)+1)]

    for j in range(len(columns)):
        results[0][j+1] = columns[j]["name"]

    for i,(name,getValues) in enumerate(values):
        scenarioValues = getValues(resultsScenario.values())
        nominalValues = getValues(resultsNominal.values())
        
        results[i+1][0] = name

        for j,c in enumerate(columns):
            f = c["func"]
            value = f(scenarioValues,nominalValues)
            
            
            if type(value) == float or type(value) == int:
                results[i+1][j+1] = "{:.2f}".format(value).rstrip('0').rstrip('.')
            else:
                results[i+1][j+1] = str(value)

    return header,results,resultsScenario


def printTableASCII(header,table):
    #Size (in characters)
    nameSize = 35
    cellSize = 13

    print("\n".join(header))

    #print("*" * (nameSize + len(columns)*(cellSize+1) + 1))
    for line in table:
        format = "{:" + str(nameSize) + "s}|" + "|".join(["{:^" + str(cellSize) + "s}" for _ in range(len(line)-1)]) + "|"
        print(format.format(*line))
    print("*" * (nameSize + len(columns)*(cellSize+1) + 1))

def printBeginLatex():
    print("""
\\documentclass{report}

\\usepackage[landscape,margin=1cm]{geometry}

\\usepackage{lmodern}
\\usepackage[T1]{fontenc}
\\usepackage[utf8]{inputenc}
\\usepackage{amsmath}
\\usepackage{amssymb}
\\usepackage[frenchb]{babel}

\\usepackage{float}
\\usepackage{array}
\\newcolumntype{M}[1]{>{\\centering\\arraybackslash}m{#1}}

\\begin{document}
    """)

def printTableLatex(header, table):

    """
    print()
    print("\\begin{center}")
    print("\n".join(header))
    print("\\end{center}")
    """
    
    
    print()
    print(r"\begin{table}[H]")
    print(r"\resizebox{\textwidth}{!}{%")
    
    table[0] = [l.replace("%", "\\%").replace("σ","$\\sigma$") for l in table[0]]
    
    for i in range(len(table)):
        for j in range(len(table[i])):
            table[i][j] = table[i][j].replace("~", r"$\sim$")
    
    print("\\begin{tabular}{|M{4cm}|" + ("M{2cm}|" *(len(table[0])-1)) + "}\\hline")
    for line in table:
        print("&".join([" {" + (columns[i-1].get("latexFont","") if i > 0 else "") + item + "} " for i,item in enumerate(line)]) + "\\\\\\hline")
    print("\end{tabular}")

    print("}")
    print(r"\caption{%s}" % header[0].split(":")[1].strip())
    print(r"\label{tab:stats:%s}" % header[0].split(":")[0].strip())
    print(r"\end{table}")
    #print("\n\\clearpage\n")

def printEndLatex():
    print("\\end{document}")

class Histo:
    def __init__(self):
        self.table = {}

    def addTable(self, header, table, data):
        self.table[header[0].split(":")[0].strip()] = (table,data)
        
    def printHisto(self):

        for indexValue in [0,10]:
            keys = []
            means = []
            errors = []
            datas = []
            
            table,data = next(iter(self.table.values()))
            keys.append("Nominal")
            means.append(float(table[indexValue+1][table[0].index("Moyenne nom.")]))
            errors.append(float(table[indexValue+1][table[0].index("σ nom.")]))
            datas.append(data)
            
            plt.axvline(float(table[indexValue+1][table[0].index("Moyenne nom.")]), color="red")

            for key in namesOrdered:
                table,data = self.table[key]
                keys.append(key)
                means.append(float(table[indexValue+1][table[0].index("Moyenne")]))
                errors.append(float(table[indexValue+1][table[0].index("Écart-type σ")]))
                datas.append(data)

            plt.barh(range(len(means)), means, xerr=errors, align='center', alpha=0.3, error_kw=dict(elinewidth=2,ecolor='blue'))

            for i in range(1,len(means)):
                valuesToDraw = values[indexValue][1](datas[i].values())
                if i == 0: print(sorted(valuesToDraw))
                if i == 0: print(sum(valuesToDraw)/len(valuesToDraw))
                plt.scatter(valuesToDraw, [i]*len(valuesToDraw), c="black", marker="x", s=70, alpha=0.8)
            
            plt.yticks(range(len(means)), list(map(lambda x: getDescription(x).replace("+","\n"), keys)))
            plt.xlabel(values[indexValue][0])
            plt.gca().set_xlim(0)
            plt.gca().set_ylim(-1, len(means))
            
            plt.gca().invert_yaxis()
            plt.tight_layout()
            
            
            
            def onresize(event):
                plt.tight_layout()

            plt.gcf().canvas.mpl_connect('resize_event', onresize)
            
            plt.show()
            
        
    
def main(argv):
    parser = argparse.ArgumentParser(description="Parse a statistical simulation")
    parser.add_argument("outputFolder"   , type=os.path.abspath)
    parser.add_argument("--logLevel"       , type=str, default="info")
    parser.add_argument("--latex"       , action="store_true")
    parser.add_argument("--full-latex", action="store_true")
    parser.add_argument("--histo", action="store_true")
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

    if args.full_latex:
        printBeginLatex()
    elif args.histo:
        histo = Histo()

    #Compute an order among the outputs
    scenariis = [d for d in os.listdir(args.outputFolder) if d.startswith("output_") and d != "output_nominal"]
    scenariis = sorted(scenariis, key=lambda x:namesOrdered.index(x.split("_")[1]) if x.split("_")[1] in namesOrdered else len(namesOrdered))

    for d in scenariis:
        h,t,data = parseScenario(os.path.join(args.outputFolder, d), os.path.join(args.outputFolder, "output_nominal"))
        if args.latex or args.full_latex:
            printTableLatex(h,t)
        elif args.histo:
            histo.addTable(h,t,data)
        else:
            printTableASCII(h,t)

    if args.full_latex:
        printEndLatex()
    elif args.histo:
        histo.printHisto()
    logger.info("Done")
            
if __name__=="__main__":
    main(sys.argv[1:])