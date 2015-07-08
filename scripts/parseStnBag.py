#!/usr/bin/env python

import sys

import rosbag
import json

def main():
    bag = rosbag.Bag(sys.argv[1])
    
    with open(sys.argv[2], "w") as f:
        for topic, msg, t in bag.read_messages(topics=["/minnie/hidden/stnupdate"]):
            f.write(json.dumps(json.loads(msg.data)) + "\n")
    
if __name__=="__main__":
    main()
