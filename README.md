# METAL: Mastn Execution with TemporAl fLexibility

METAL is a distributed robotics supervisor : its goal is to allow a team of robots to execute a plan. Each robot runs an instance of METAL. All those instances communicate together to synchronise the plan execution.

METAL depends on the ROS middleware for all its communication. It depends on Qt4 for its graphical interface (optionnal).

## Use

### METAL

METAL is mainly launched with a launch file associated with a mission. An exemple is in "data/simple-mission". More complex missions are created with SIMATO. "launch/simple.launch" is the launch file associated with the simple mission.

To run a simulation with all robots runnning of the host :

    $ roslaunch mission.launch

To launch a mission with different robot :
    $ roslaunch simu:=false #on the operator computer
    $ roslaunch robottype.launch agentName:=robotname

Another option allow to run all the robots supervisor on a different machine than the one running the ROS master, visualtion, etc. It allows to reduce the load on this computer. The available hosts are defined in "launch/machine.launch"

    $ roslaunch mission.launch machine_multi:=PC-Action-Fixe

When all the supervisor are online, if auto_start is not set and all the METAL where given the --waitSignal flag, the mission is launched with :

    $ rostopic pub /hidden/start std_msgs/Empty "{}" -1


### Sending aleas

When running, each supervisor accepts external commands. This is use to simulate some unexpected events. Exemple of individual commands are given in "script/exemples_commandes.txt"

Alea can also be defined in batch. This allows the alea to be reliably sent : they will be send at a precise date (relative to the beginning of the plan). Sample alea files are found in "data/aleas-simple" or in "data/aleas-V".

### Timeline

Timeline listen to ROS topic to display the plan being run by each robot and its internal state. It can be given the mission file to use the same set of color for the team as other tools.

    $ rosrun metal onlineTimeline.py --mission path/to/mission.json

### Statistical simultations

All the code for launching and parsing statistical simulation is found in the "data/stats" folder. "simu_stats.py" runs individual simulation (called simu) of sets of individual simulation (called benchmark). "benchmarkGenerator.py" will randomly generate the alea files and call "simu_stat.py" for each pattern. "benchmarkParser.py" will parse the output of a benchmark.

When run, "benchmarkGenetator" will put all the generated file in the given outputFolder.

Several patterns for aleas are defined. When "benchmarkGenetator" is called, it will iterate through them (or a subset, defined at the end of the script). For each of them it will create a set of aleas (in the "aleas_pattern" folder") and then call simu_stats to run them. It will create folders "output_pattern/simu_i" with i being the index of the run. The alea file used will be copied in this folder. It will also contains a bag of all the transmitted messages, the output of all the supervisors, etc. 

/!\ It can take a lot of space (about 1Go for 30 runs).

The option for "benchmarkGenetator" are :

* "outputFolder" : where to put the generated files. Will create the folder
* "mission" : the path to the mission to run. The mission folder should have already been created by SIMATO.
* "nbrInstance" : number of aleas to create per pattern
* "jobs" : the number of simulation that can be runned in parallel. Recommended to 3.
* "logLevel" : level of log to use for this script
* "dry-run" : only create the aleas files, do not run the simulations

Once the simulations are done, "benchmarkParser.py" will analyse them (replaying the bag file) and compute several statistics. It can output the result in several ways : as ASCII in the standard output, as a latex table (--latex flag), as a self-contained latex document (--full-latex flag) or as histograms using matplotlib (--histo flag).

#### Exemple

    $ ./benchmarkGenerator.py --outputFolder /path/to/metal/simu_output/test --mission path/to/mission.json --nbrInstance 30 --jobs 3
    $ ./benchmarkParser.py /path/to/metal/simu_output/test
    $ ./benchmarkParser.py --histo --output outputFolder /path/to/metal/simu_output/test

## Project Structure

* "scripts" : contains all the scripts for the project
    * "executors" : contains the code specific to each target robot (each being a subclass of executor.py)
    * "stats" : contains the code needed to run statistical simultations
* "srv" and "msg" : definition of the ROS messages and services
* "launch" : ROS launch files (including for all the target robots)
* "data" : Hold several data file. Including sample alea files, plans, etc.

## Code Structure

When launched, METAL launchs tow thread : the supervisor and the executor. The executor if responsible of the communication with the robot. It receive orders from the supervisor, translate them for the specific robot it runs on and send them to the robot. When it receives a status update it sends it to the supervisor.

The code for METAL is split into several files :

* "hidden.py" and "hidden_ros.py" : the main entry point for METAL (still named after the previous project)
* "supervisor.py" and "supervisor_ros.py" : The latter hold all the ROS-specific functions.
* "plan.py" : define a class to deal with JSON plan object. Can read/write them and convert them to a format usable by METAL. In particular the timepoints are renamed : instead of just using a timepoint they are named "index-t-action" where "t" is either "start" or "end" and "action" is the name of the action associated to this timepoint

METAL also include a graphical visualisation of the internal state of each robot, called Timeline :

* "onlineTimeline.py" is the main code for this.
* "plotWindow.py" hold some Qt-related definition for the main window.

METAL also comes with ease-of-life scripts.

* "aleaInjector.py" : mainly used in statistical simulation. It expect as a input an alea file. This file describe the list of aleas that will be launched. Each alea is associated with a time, a robot, a type and some other information.
* "autoStart.py" : read the list of all agents from the plan. Launches the mission when all robots have reported to be ready (ie. in the INIT state)
* "checkLiveness.py" : ping a list of hosts to check which one are still alive. Useful during the real-life experiments
* "createHybridLaunch.py" : used to create a launch file for hybrid simulation. Given the path to a mission directory created by SIMATO, will create a specific launch file with a set of real robots and a set of simulated robots.
* "watcher.py" : monitor the state of each robot. It will kill itself when any robot is in ERROR state or when all robots are done (in DONE, TRACKING or DEAD). Used as a required process in launch files