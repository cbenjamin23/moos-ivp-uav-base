#!/bin/bash


mkdir -p simulation

cd ./simulation
# Start Gazebo in a new terminator window
# terminator --new-tab -e "bash -l -c 'source ~/.profile; source ~/.bashrc; sleep 2;echo \$BASH_SOURCE; env; gz sim -v4 -r skywalker_x8_runway.sdf; exec bash'" &
nohup gz sim -v4 -r skywalker_x8_runway.sdf &> /dev/null &

# Start Mission Planner in the background in a third terminator window
# terminator --new-tab -e "bash -c 'source ~/.profile; cd ~/MissionPlanner; mono MissionPlanner.exe; exec bash'" &
# cd ~/Mission_Planner
# nohup mono MissionPlanner.exe &> /dev/null &
# cd -


# Start ArduPilot SITL in another terminator window
#  xterm -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map; exec bash'" &
# terminator --new-tab -e "bash -c 'source ~/.profile; sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map; exec bash'" &

DEBUG=OFF

# if argument -gdb is given, then start gdb
for ARGI; do
    if [ "$ARGI" == "--gdb" ]; then
        DEBUG=ON
    fi
done




if [ $DEBUG == "ON" ]; then
    echo "Starting ArduPilot SITL in GDB mode"
    gdb --args python /home/steve/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file=mav.parm  #--no-mavproxy &
else
    echo "Starting ArduPilot SITL in normal mode"
    sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file=mav.parm  #--no-mavproxy &
fi

