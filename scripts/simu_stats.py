#! /usr/bin/env python
### rosbag API is not python3 compatible ###

import argparse
import json
import logging; logger = logging.getLogger("simu")
import os
import re
import shutil
import subprocess
import sys
import time

import rospkg
rospack = rospkg.RosPack()
import rosbag
from std_msgs.msg import Empty, String
from metal.msg import StnVisu


sh = logging.StreamHandler()
logger.setLevel(logging.INFO)
sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
logger.addHandler(sh)

class InputError(Exception):
    pass

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
def launchSimu(missionDir, aleaFile, outputDir):
    os.chdir(missionDir)

    #run the rolaunch command
    os.environ["ROS_LOG_DIR"] = outputDir
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11312"
    p = subprocess.Popen("roslaunch --run_id=roslaunch stats_simu.launch alea_file:={alea}".format(alea=aleaFile).split(" "))

    time.sleep(5)

    subprocess.call("rostopic pub /hidden/start std_msgs/Empty -1".split(" "))

    return p

def runSimu(missionDir, aleaFile, outputDir = None):
    outputDir = configureSimu(missionDir, aleaFile, outputDir)

    p = launchSimu(missionDir, aleaFile, outputDir)
    p.wait()

    parseSimu(outputDir)
    
    logger.info("Done with this simulation : %s" % outputDir)

"""
Run a succession of simulations. For now, sequentially.
"""
def runBenchmark(missionDir, aleaFiles, outputDir = None):

    #create the output dir
    if outputDir is None:
        outputDir = getNewOutputDir(prefix = "benchmark")

    if not os.path.exists(outputDir):
        os.makedirs(outputDir)

    for i,alea in enumerate(aleaFiles):
        runSimu(missionDir, alea, outputDir= os.path.join(outputDir, "simu_%d" % i))

    logger.info("Done with this benchmark : %s" % outputDir)

    #TODO merge all the results of the simulations

"""
Parse the ros bag and extract metrics :
 - Success of the mission
 - Number of points explored
 - Duration of the mission
"""
def parseSimu(outputDir):
    logger.info("Parsing %s" % outputDir)
    result = {"obsPoints" : {}, "repair":{}}
    
    ## Parse the pddl files ##
    
    pddlPrb = None
    for f in os.listdir(os.path.join(outputDir, "hipop-files")):
        if re.match(".*-prb.pddl", f):
            pddlPrb = f
            break #assume there is only one file for this pattern

    with open(os.path.join(outputDir, "hipop-files", pddlPrb)) as f:
        strPrb = " ".join(f.readlines())
        #TODO : replace("\n", " ")

    obsPointsNominal = re.findall("(explored [^)]*)", strPrb)
    result["obsPoints"]["nominalNbr"] = len(obsPointsNominal)

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
        agents = {}
        for _,msg,_ in bag.read_messages(topics="/hidden/stnvisu"):
            if msg.agent not in agents:
                agents[msg.agent] = {}
            
            if "finishTime" not in agents[msg.agent] and msg.state in ["DONE", "ERROR", "DEAD"]:
                agents[msg.agent]["finishTime"] = msg.time
                
            if msg.state == "ERROR":
                hasError = True
            
            logger.debug("%s : %s -> %s" % (msg.time, msg.agent, msg.state))

        for agent,d in agents.items():
            if "finishTime" not in d:
                logger.error("Cannot determine the end time of %s" % agent)

        result["success"] = not hasError
        result["finishTime"] = max([d["finishTime"]/1000. for d in agents.values() if "finishTime" in d])
        

        repairRequestNbr = 0
        repairDoneNbr = 0
        for _,msg,_ in bag.read_messages(topics="/hidden/repair"):
            if msg.type == "repairRequest": repairRequestNbr += 1
            if msg.type == "repairDone": repairDoneNbr += 1

        result["repair"]["requestNbr"] = repairRequestNbr
        result["repair"]["doneNbr"] = repairDoneNbr


    finally:
        bag.close()

    with open(os.path.join(outputDir, "results.json"), "w") as f:
        json.dump(result, f, indent=2)

def main(argv):
    parser = argparse.ArgumentParser(description="Launch a statistical simulation")
    parser.add_argument("-a", "--aleaFiles", type=os.path.abspath, nargs="+")
    parser.add_argument("-m", "--mission"  , type=os.path.abspath)
    parser.add_argument("--parseOnly"      , action="store_true")
    parser.add_argument("--outputFolder"   , type=os.path.abspath)
    parser.add_argument("--logLevel"       , type=str, default="info")
    args = parser.parse_args(argv)

    ## Parse only ##
    if args.parseOnly:
        if args.outputFolder is None:
            logger.error("Cannot parse. No given output folder")
            sys.exit(1)
        if not os.path.exists(args.outputFolder):
            logger.error("Cannot open the output folder : %s" % args.outputFolder)
            sys.exit(1)
    
        parseSimu(args.outputFolder)
        sys.exit(0)

    ## Run simulation ##
    if args.aleaFiles is None or args.mission is None:
        logger.error("You must specify at least one alea file and the mission folder")
        sys.exit(1)

    if not os.path.isdir(args.mission):
        logger.error("The mission folder does not exists : %s" % args.mission)
        sys.exit(1)

    for alea in args.aleaFiles:
        if not os.access(alea, os.R_OK):
            logger.error("Cannot open a given alea file : %s" % alea)
            sys.exit(1)

    if len(args.aleaFiles) == 1:
        runSimu(args.mission, args.aleaFiles[0], args.outputFolder)
    else:
        runBenchmark(args.mission, args.aleaFiles, args.outputFolder)

if __name__=="__main__":
    main(sys.argv[1:])
