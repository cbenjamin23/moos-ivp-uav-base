#!/bin/bash

DEBUG=OFF
MISSIONPLANNER=OFF
PARAMFILE=sim.parm
# if argument -gdb is given, then start gdb
for ARGI; do
    if [[ "$ARGI" == "--gdb" ]]; then
        DEBUG=ON
    elif [[ "$ARGI" == "--mp" ]]; then
        MISSIONPLANNER=ON
    elif [[ "$ARGI" == --parm=* ]]; then
        PARAMFILE="${ARGI#--parm=}"
    else
        echo "Unknown argument: $ARGI"
        # print help
        echo "Usage: start_simulation.sh [--gdb] [--mp] [--parm=filename]"
        exit 1
    fi
done


PARAMFILE=$HOME/moos-ivp-uav/scripts/simulation/$PARAMFILE


mkdir -p simulation

cd ./simulation
# Start Gazebo in a new terminator window
# terminator --new-tab -e "bash -l -c 'source ~/.profile; source ~/.bashrc; sleep 2;echo \$BASH_SOURCE; env; gz sim -v4 -r skywalker_x8_runway.sdf; exec bash'" &
nohup gz sim -v4 -r skywalker_x8_runway.sdf &> /dev/null &

if [ $MISSIONPLANNER == "ON" ]; then
    # Start Mission Planner in the background in a third terminator window
    echo "Starting Mission Planner"
    # Start in new xterm window
    xterm -e "bash -c 'source ~/.profile; cd ~/MissionPlanner; mono MissionPlanner.exe; exec bash'" &
    
    # terminator  -e "bash -c 'source ~/.profile; cd ~/MissionPlanner; mono MissionPlanner.exe; exec bash'" &
    
    # cd ~/Mission_Planner
    # nohup mono MissionPlanner.exe &> /dev/null &
    # cd -
fi


# Start ArduPilot SITL in another terminator window
#  xterm -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map; exec bash'" &
# terminator --new-tab -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map; exec bash'" &



cd $HOME/ardupilot/ArduPlane

if [ $DEBUG == "ON" ]; then
    echo "Starting ArduPilot SITL in GDB mode"
    gdb --args python $HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file="$PARAMFILE" #--no-mavproxy &
else
    echo "Starting ArduPilot SITL in normal mode"
    sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file="$PARAMFILE"  #--no-mavproxy &
    # sim_vehicle.py -v ArduPlane -f gazebo-zephyr  --console --map --add-param-file="$PARAMFILE" --count 2  --swarm Tools/autotest/swarminit.txt #--no-mavproxy &
fi


