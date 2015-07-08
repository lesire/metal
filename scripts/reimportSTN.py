#!/usr/bin/env python3

import argparse
import json
import logging
import subprocess
import sys
import pystn

    
def main():
    r = subprocess.call(("rosrun metal parseStnBag.py %s /tmp/stn_temp.txt" % sys.argv[1]).split(" "))
    logging.info("Parsing done : %s" % r)

    s = None
    with open("/tmp/stn_temp.txt") as f:
        for msg in f:
            data = json.loads(msg)
            bounds = None
            export = None
            
            if s is not None and not s.isConsistent():
                logging.error("STN not consistent when beginning")

            if "init" in data:
                if s is not None:
                    logging.warning("Reinitialisation of the stn")
                    
                s = pystn.importSTNIntMa(data["init"], pystn.StnType.GraphSTN)
            elif "start" in data and "end" in data:
                bounds = s.getConstraint(data["start"], data["end"])
                export = s.export()
                
                if data["type"] == "set":
                    func = s.setConstraint
                elif data["type"] == "add":
                    func = s.addConstraint
                else:
                    logging.error("Cannot find the type of the data %s" % data)
                    
                    
                if "lb" in data and "ub" in data:
                    func(data["start"], data["end"], data["lb"], data["ub"])
                elif "lb" in data:
                    func(data["start"], data["end"], data["lb"])
                else:
                    logger.error("Cannot parse %s" % data)
            elif "mastnUpdate" in data:
                export = s.export(),s.isConsistent()
            
                cl = pystn.ConstraintListL()
                for d in data["mastnUpdate"]:
                    cl.append(pystn.ConstraintInt(d["start"], d["end"], d["lb"], d["ub"]))

                s.setConstraints(cl)
            else:
                logging.error(list(data.keys()))
                
            if s is None:
                logging.warning("STN is not defined")
            else:
                if not s.isConsistent():
                    logging.warning("STN is inconsistent")
                    logging.warning(list(data.keys()))
                    logging.warning(data)
                    logging.warning(bounds)
                    logging.warning(export)
                    sys.exit(1)

if __name__=="__main__":
    main()
