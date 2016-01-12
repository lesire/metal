#!/usr/bin/env python3

"""
Expect a mission folder created by SIMATO (with a mission.launch and a run_morse.py) and a list of real or simulated robots
Will create a "mission_hybrid.launch" with only the simulated robots (and the real ones in the morse file, listening for the pose send from the real robots)

"""

import re
import argparse
import os
import sys
import json

def main():
    parser = argparse.ArgumentParser(description='Create a launch file for an hybrid run in the ACTION project.')
    parser.add_argument('--real', type=str, nargs='+', metavar='robot', help='the list of real robots')
    parser.add_argument('--simulated', type=str, nargs='+', metavar='robot', help='the list of simulated robots')
    parser.add_argument('--wd', default='.', type=str)
    args = parser.parse_args()
    
    print("Working directory : %s" % args.wd)
    
    if "mission.launch" not in os.listdir(args.wd):
        print("Cannot find mission.launch. Exiting")
        sys.exit(1)
    
    if "run_morse.py" not in os.listdir(args.wd):
        print("Cannot find run_morse.py. Exiting")
        sys.exit(1)

    os.chdir(args.wd)
    
    missionFile = os.path.basename(os.getcwd()) + ".json"
    with open("../" + missionFile) as f:
        mission = json.load(f)
        robots = list(mission["agents"].keys())
    
    print("Known robots are : %s" % " ".join(robots))
    simulated = []
    real = []

    if args.real is None and args.simulated is None:
        print("Error : no real or simulated robots defined")
        sys.exit(1)
    elif args.real is None:
        for r in robots:
            if r in args.simulated:
                simulated.append(r)
            else:
                real.append(r)
    elif args.simulated is None:
        for r in robots:
            if r in args.real:
                real.append(r)
            else:
                simulated.append(r)
    else:
        print("Both real and simulated defined. Not implemented yet")
        sys.exit(1)

    print("%s are real" % " ".join(real))
    print("%s are simulated" % " ".join(simulated))

    with open("run_morse_hybrid.py", "w") as f:
        with open("run_morse.py", "r") as f_in:
            for line in f_in:
            
                m = re.match("([a-zA-Z0-9]+)\s*=\s*[a-zA-Z0-9]+\([^#]*#[^#]*\)", line)
                if m:
                    robot = m.groups()[0]
                    
                    if robot in real:
                        line = line.replace(") #", "")

                f.write(line)

    with open("mission_hybrid.launch", "w") as f:
        with open("mission.launch", "r") as f_in:
            skip = False

            for line in f_in:
                m = re.match("\s*<include.*/([a-zA-Z0-9]*)\.launch\">", line)
                if m:
                    robot = m.groups()[0]
                    if robot in real:
                        skip = True
                        continue
                if re.match("\s*</include>\s*", line):
                    if skip:
                        skip = False
                        continue

                if "run_morse.py" in line:
                    line = line.replace("run_morse.py", "run_morse_hybrid.py")

                if not skip:
                    f.write(line)

    print("Done")

if __name__=="__main__":
    main()
