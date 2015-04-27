#!/bin/sh

# sourcer ~/.profile
# sourcer ~/work/openrobots/bin/libRessac-Env

# A executer dans le dossier scripts

./hidden.py --agentName ressac1 --executor 'libRessac' --waitSignal ../data/action-V.plan 
