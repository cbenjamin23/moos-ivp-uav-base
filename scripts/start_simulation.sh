#!/bin/bash

# Start Gazebo in a new terminator window
# terminator --new-tab -e "bash -l -c 'source ~/.profile; source ~/.bashrc; sleep 2;echo \$BASH_SOURCE; env; gz sim -v4 -r skywalker_x8_runway.sdf; exec bash'" &
nohup gz sim -v4 -r skywalker_x8_runway.sdf &> /dev/null &

# Start ArduPilot SITL in another terminator window
terminator --new-tab -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map; exec bash'" &

# terminator --new-tab -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane -A "--serial0=udpclient<127.0.0.1>:14550" --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --no-mavproxy; exec bash'" &

# Start Mission Planner in the background in a third terminator window
terminator --new-tab -e "bash -c 'source ~/.profile; cd ~/Mission_Planner; mono MissionPlanner.exe; exec bash'" &
# cd ~/Mission_Planner
# nohup mono MissionPlanner.exe &> /dev/null &
# cd -

