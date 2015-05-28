#! /usr/bin/env python3

import logging; logger = logging.getLogger("simu")
import os
import shutil
import subprocess
import time

import rospy
import rospkg
rospack = rospkg.RosPack()
from std_msgs.msg import Empty

sh = logging.StreamHandler()
logger.setLevel(logging.INFO)
sh.setFormatter(logging.Formatter('[%(asctime)s] %(levelname)s (%(filename)s:%(lineno)d): %(message)s'))
logger.addHandler(sh)


class InputError(Exception):
    pass

def getNewOutputDir():
    return os.path.join(rospack.get_path("metal"), "data/simu_output/" + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()))

def runSimu(missionDir, aleaFile, outputDir = None):
    # Sanity check
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
        outputDir = getNewOutputDir()

    if not os.path.exists(outputDir):
        os.makedirs(outputDir)

    logger.info("Output dir for simu is %s" % outputDir)

    os.chdir(missionDir)

    #copy input files
    shutil.copytree("hipop-files", os.path.join(outputDir, "hipop-files"))
    shutil.copy(aleaFile, outputDir)

    #run the rolaunch command
    os.environ["ROS_LOG_DIR"] = outputDir
    p = subprocess.Popen("roslaunch --run_id=roslaunch stats_simu.launch alea_file:={alea} visu:=true".format(alea=aleaFile).split(" "))

    time.sleep(5)

    subprocess.call("rostopic pub /hidden/start std_msgs/Empty -1".split(" "))

    #TODO add a timeout or make the watcher detect deadlocks
    p.wait()

    #TODO parse the output

    logger.info("done with this simulation")

def main():

    runSimu("/Users/patrick/Documents/workspace_planner/ressources/missions/odas-mission", "/Users/patrick/Documents/workspace_planner/action_ros_ws/src/metal/data/delay-odas.json")

if __name__=="__main__":
    main()