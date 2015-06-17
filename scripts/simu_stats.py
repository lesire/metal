#! /usr/bin/env python
### rosbag API is not python3 compatible ###

from __future__ import division

import argparse
from copy import copy
import json
import logging; logger = logging.getLogger(__name__)
import os
import re
import shutil
import signal
import subprocess
import sys
import time

import rospkg
rospack = rospkg.RosPack()
import rosbag
from std_msgs.msg import Empty, String
from metal.msg import StnVisu

class InputError(Exception):
    pass

############### Signal Handling ######################
"""
To stop the creation of new processes, you can use :
  kill -SIGUSR1 pID
to send the SIGUSR1 signal
"""
#once this signal is received, stop creating threads
sigusrReceived = False

def sigusrHandler(signum, frame):
    global sigusrReceived
    logger.info("Received the signal : not creating new processes any more")
    sigusrReceived = True

signal.signal(signal.SIGUSR1, sigusrHandler)
#######################################################

def mean(l):
    return sum(l)/len(l)

def getPlanLength(planPDDLFile):
    result = 0
    with open(planPDDLFile) as f:
        for line in f:
            if line.startswith(";"):
                continue

            m = re.match("^(\d*(?:.\d*)?)\s*:\s*\((.*)\)\s*\[(\d*(?:.\d*)?)\]", line)
            if m:
                endAction = float(m.groups()[0]) + float(m.groups()[2])
                result = max(result, endAction)
            
    return result

def getNewOutputDir(prefix = "simu"):
    return os.path.join(rospack.get_path("metal"), "data/simu_output/" + prefix + "_" + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()))

"""
Check the input and configure the simulation : creates the output folder, 
copy the input files, etc.
"""
def configureSimu(missionDir, aleaFile, outputDir = None):
    if not os.path.exists(missionDir):
        logger.error("The mission directory does not exists : %s" % missionDir)
        raise InputError("The mission directory does not exists : %s" % missionDir)

    if not os.path.exists(os.path.join(missionDir, "stats_simu.launch")):
        logger.error("Cannot find stats_simu.launch in %s" % missionDir)
        raise InputError("Cannot find stats_simu.launch in %s" % missionDir)

    if not os.path.exists(os.path.join(missionDir, "hipop-files")):
        logger.error("Cannot find hipop-files in %s" % missionDir)
        raise InputError("Cannot find hipop-files in %s" % missionDir)

    if not os.path.exists(aleaFile):
        logger.error("Cannot find the alea file : %s" % aleaFile)
        raise InputError("Cannot find the alea file : %s" % aleaFile)
        
    #create the output dir
    if outputDir is None:
        outputDir = getNewOutputDir(prefix = "simu")

    if not os.path.exists(outputDir):
        os.makedirs(outputDir)

    logger.info("Output dir for simu is %s" % outputDir)

    os.chdir(missionDir)

    #copy input files
    shutil.copytree("hipop-files", os.path.join(outputDir, "hipop-files"))
    shutil.copy(aleaFile, outputDir)

    return outputDir

"""
Launch a simuation and returns a Popen object
"""
def launchSimu(missionDir, aleaFile, outputDir, port = 11311, redirectOutput = False, visu = False, vnet = False):
    os.chdir(missionDir)
    
    stdout = None
    stderr = None
    if redirectOutput:
        stdout = open(os.path.join(outputDir, "roslaunch-out.txt"), 'w')
        stderr = open(os.path.join(outputDir, "roslaunch-err.txt"), 'w')


    #run the rolaunch command
    os.environ["ROS_LOG_DIR"] = outputDir
    os.environ["ROS_HOME"] = outputDir
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" %  port
    logger.info("Launching a simulation on port %s" % port)
    p = subprocess.Popen("roslaunch --run_id=roslaunch stats_simu.launch alea_file:={alea} vnet:={vnet} visu:={visu} auto_start:=true"
                         .format(alea=aleaFile,
                                 visu="true" if visu else "false",
                                 vnet="true" if vnet else "false").split(" "),
                                 stdout=stdout, stderr=stderr)

    time.sleep(5)

    return p

def runSimu(missionDir, aleaFile, outputDir = None, visu = False, vnet = False):
    outputDir = configureSimu(missionDir, aleaFile, outputDir)

    p = launchSimu(missionDir, aleaFile, outputDir, visu=visu, vnet=vnet)
    p.wait()

    parseSimu(outputDir)
    
    logger.info("Done with this simulation : %s" % outputDir)

"""
Inputs is a list of dictionnaries, each containing the missionDir, aleaFile and an option outputDir
"""
def runParallelSimu(inputs, maxJobs = 1, vnet=False):

    # Configure all the simus
    nextPort = 11312
    for d in inputs:
        d["outputDir"] = configureSimu(d["missionDir"], d["aleaFile"], d.get("outputDir", None))
        d["port"] = nextPort
        nextPort += 1

    # Launch all the simus
    simuToLaunch = copy(inputs)
    processes = []
    while (len(simuToLaunch) > 0 and not sigusrReceived) or len(processes) > 0:
        #still work to do. First launch new processes
        if len(processes) < maxJobs and len(simuToLaunch) > 0 and not sigusrReceived:
            input = simuToLaunch[0]
            processes.append((input["outputDir"], launchSimu(input["missionDir"], input["aleaFile"], input["outputDir"], port=input["port"], redirectOutput=True, vnet=vnet)))
            del simuToLaunch[0]

        # Now check for the end of the launched processes
        # Iterate backward to remove elements while iterating
        for i in reversed(range(len(processes))):
            name,p = processes[i]
            p.poll()
            if(p.returncode != None):
                #process finished
                logger.info("Process %s finished with error code %s" % (name,p.returncode))
                del processes[i]

        time.sleep(1)

    # Parse all the simus
    for d in inputs:
        parseSimu(d["outputDir"])


"""
Run a succession of simulations. For now, sequentially.
"""
def runBenchmark(missionDir, aleaFiles, outputDir = None, maxJobs = 1, vnet = False):

    #create the output dir
    if outputDir is None:
        outputDir = getNewOutputDir(prefix = "benchmark")

    if not os.path.exists(outputDir):
        os.makedirs(outputDir)

    #for i,alea in enumerate(aleaFiles):
    #    runSimu(missionDir, alea, outputDir= os.path.join(outputDir, "simu_%d" % i))

    inputs = []
    for i,alea in enumerate(aleaFiles):
        inputs.append({"missionDir" : missionDir, "aleaFile" :  alea, "outputDir":os.path.join(outputDir, "simu_%d" % i)})
    runParallelSimu(inputs, maxJobs=maxJobs, vnet=vnet)
    
    logger.info("Done with this benchmark : %s" % outputDir)

    #TODO merge all the results of the simulations
    parseBenchmark(outputDir)

def parseBenchmark(outputDir):
    
    #rerun the parsing in case of an update
    results = {}
    for dir in sorted(os.listdir(outputDir)):
        if re.match("^simu_\d+$", dir):
            parseSimu(os.path.join(outputDir, dir))
            
            with open(os.path.join(outputDir, dir, "results.json")) as f:
                data = json.load(f)
                
            results[dir] = data
            
    logger.info("Found %s runs" % len(results))
    
    i = len([v for v in results.values() if v["success"]])
    logger.info("Found %d successes : %.2f%%" % (i, 100*i/len(results)))
    
    lengths = [v["finishTime"] for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean time (for the sucessful missions) : %.2f" % (m))
    
    lengths = [(v["finishTime"]/v["nominalTime"] - 1) for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean increase of time (for the sucessful missions) : %.2f%%" % (100*m))
    
    lengths = [v["obsPoints"]["nbr"] for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean number of points explored (for the sucessful missions) : %.2f" % (m))
    
    lengths = [v["obsPoints"]["ratio"] for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean percentage of points explored (for the sucessful missions) : %.2f%%" % (100*m))
    
    lengths = [v["repair"]["requestNbr"] for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean number of repair requests (for the sucessful missions) : %.2f" % (m))
    
    lengths = [v["repair"]["totalTime"] for v in results.values() if v["success"]]
    m = mean(lengths)
    logger.info("Mean time taken to repair the plan (sec) (for the sucessful missions) : %.2f" % (m/1000))

    pass
    
"""
Parse the ros bag and extract metrics :
 - Success of the mission
 - Number of points explored
 - Duration of the mission
"""
def parseSimu(outputDir):
    logger.info("Parsing %s" % outputDir)
    result = {"obsPoints" : {}, "repair":{}}
    
    if not os.path.exists(os.path.join(outputDir, "stats.bag")):
        logger.error("No simulation ran in %s" % outputDir)
        return
    
    
    ## Parse the pddl files ##
    
    missionName = None
    pddlPrb = None
    for f in os.listdir(os.path.join(outputDir, "hipop-files")):
        if re.match(".*-prb.pddl", f):
            pddlPrb = f
            break #assume there is only one file for this pattern

    missionName = pddlPrb.replace("-prb.pddl", "")
    with open(os.path.join(outputDir, "hipop-files", pddlPrb)) as f:
        strPrb = " ".join(f.readlines())
        #TODO : replace("\n", " ")

    obsPointsNominal = re.findall("(explored [^)]*)", strPrb)
    result["obsPoints"]["nominalNbr"] = len(obsPointsNominal)

    
    result["nominalTime"] = getPlanLength(os.path.join(outputDir, "hipop-files", missionName + ".pddl"))
    ## Parse the ros bag ##
    
    bag = rosbag.Bag(os.path.join(outputDir, "stats.bag"))
    
    try:
        logger.info("Number of messages %s" % bag.get_message_count())
        
        obsPoints = set()
        for _, msg, _ in bag.read_messages(topics="/hidden/stats"):
            data = json.loads(msg.data)
            
            if data["type"] == "observe":
                obsPoints.add(data["to"])
            
            logger.debug(data)

        result["obsPoints"]["nbr"] = len(obsPoints)
        result["obsPoints"]["ratio"] = float(result["obsPoints"]["nbr"]) / result["obsPoints"]["nominalNbr"]

        hasError = False
        trackingRobots = set()
        agents = {}
        for _,msg,_ in bag.read_messages(topics="/hidden/stnvisu"):
            if msg.agent not in agents:
                agents[msg.agent] = {}
            
            if "finishTime" not in agents[msg.agent] and msg.state in ["DONE", "ERROR", "DEAD", "TRACKING"]:
                agents[msg.agent]["finishTime"] = msg.time
                
            if msg.state == "ERROR":
                hasError = True
                
            if msg.state == "TRACKING":
                trackingRobots.add(msg.agent)
            
            logger.debug("%s : %s -> %s" % (msg.time, msg.agent, msg.state))

        for agent,d in agents.items():
            if "finishTime" not in d:
                logger.error("Cannot determine the end time of %s" % agent)
                hasError = True

        result["success"] = not hasError
        result["trackingRobots"] = list(trackingRobots)
        result["trackingRobotsNbr"] = len(trackingRobots)

        l = [d["finishTime"]/1000. for d in agents.values() if "finishTime" in d]
        if l:
            result["finishTime"] = max(l)

        repairRequestNbr = 0
        repairDoneNbr = 0
        repairLengths = []
        repairBegin = None
        for _,msg,_ in bag.read_messages(topics="/hidden/repair"):
            if msg.type == "repairRequest":
                repairRequestNbr += 1
                repairBegin = int(msg.time)
            if msg.type == "repairDone":
                repairDoneNbr += 1
                if repairBegin is None:
                    print("Error : repairDone sent before repairRequest ?")
                else:
                    repairLengths.append(int(msg.time) - repairBegin)
                    repairBegin = None

        result["repair"]["requestNbr"] = repairRequestNbr
        result["repair"]["doneNbr"] = repairDoneNbr
        result["repair"]["lengths"] = repairLengths
        result["repair"]["totalTime"] = sum(repairLengths)


        droppedComs = set()
        for _,msg,_ in bag.read_messages(topics="/hidden/mastnUpdate"):
            droppedComs = droppedComs.union(set(msg.droppedComs))

        result["droppedComs"] = len(droppedComs)

    finally:
        bag.close()

    with open(os.path.join(outputDir, "results.json"), "w") as f:
        json.dump(result, f, indent=2)

def main(argv):
    parser = argparse.ArgumentParser(description="Launch a statistical simulation")
    parser.add_argument("-a", "--aleaFiles", type=os.path.abspath, nargs="+")
    parser.add_argument("-m", "--mission"  , type=os.path.abspath)
    parser.add_argument("-j", "--jobs"     , type=int, default=1)
    parser.add_argument("--parseOnly"      , action="store_true")
    parser.add_argument("--outputFolder"   , type=os.path.abspath)
    parser.add_argument("--visu"           , action="store_true")
    parser.add_argument("--vnet"           , action="store_true")
    parser.add_argument("--logLevel"       , type=str, default="info")
    args = parser.parse_args(argv)

    #Configure the logger
    numeric_level = getattr(logging, args.logLevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.logLevel)
    logger.propagate = False
    sh = logging.StreamHandler()
    logger.setLevel(numeric_level)
    sh.setFormatter(logging.Formatter('%(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
    logger.addHandler(sh)
        
    ## Parse only ##
    if args.parseOnly:
        if args.outputFolder is None:
            logger.error("Cannot parse. No given output folder")
            sys.exit(1)
        if not os.path.exists(args.outputFolder):
            logger.error("Cannot open the output folder : %s" % args.outputFolder)
            sys.exit(1)
    
        if "simu_0" in os.listdir(args.outputFolder):
            parseBenchmark(args.outputFolder)
        elif "stats.bag" in os.listdir(args.outputFolder):
            parseSimu(args.outputFolder)
        else:
            logger.error("Cannot determine if %s is a simulation or a benchmark" % args.outputFolder)
        sys.exit(0)

    ## Run simulation ##
    if args.aleaFiles is None or args.mission is None:
        logger.error("You must specify at least one alea file and the mission folder")
        sys.exit(1)

    if not os.path.isdir(args.mission):
        logger.error("The mission folder does not exists : %s" % args.mission)
        sys.exit(1)

    if len(args.aleaFiles) == 1 and not os.path.isdir(args.aleaFiles[0]):
        #run simu with this single alea file
        alea = args.aleaFiles[0]
        if not os.access(alea, os.R_OK):
            logger.error("Cannot open a given alea file : %s" % alea)
            sys.exit(1)
        runSimu(args.mission, args.aleaFiles[0], args.outputFolder, visu=args.visu, vnet=args.vnet)
    else:
        if len(args.aleaFiles) == 1 and os.path.isdir(args.aleaFiles[0]):
            #run benchmark on all json files in this directory
            aleaFiles = [os.path.join(args.aleaFiles[0], f) for f in sorted(os.listdir(args.aleaFiles[0])) if f.endswith(".json")]
        else:
            aleaFiles = args.aleaFiles

        for alea in aleaFiles:
            if not os.access(alea, os.R_OK):
                logger.error("Cannot open a given alea file : %s" % alea)
                sys.exit(1)

        runBenchmark(args.mission, aleaFiles, args.outputFolder, maxJobs=args.jobs, vnet=args.vnet)

if __name__=="__main__":
    main(sys.argv[1:])
