#! /usr/bin/env bash
DIR=$( cd "$( dirname "${{BASH_SOURCE[0]}}" )" && pwd )
cd $DIR

hipop --logLevel error -I simple-prb-init.plan -P hadd_time_lifo -A areuse_motion_nocostmotion -F local_openEarliestMostCostFirst_motionLast -O simple.pddl -o simple.plan simple-domain.pddl simple-prb.pddl
