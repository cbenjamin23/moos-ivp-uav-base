#!/bin/bash

DEBUG=OFF
MISSIONPLANNER=OFF
PARAMFILE=sim.parm

NUM_VEHICLES=1


GZ_ONLY=OFF

# Parse arguments
for (( i=1; i <= $#; i++ )); do
    ARGI="${!i}"
    case "$ARGI" in
        --gdb)
            DEBUG=ON
            ;;
        --mp)
            MISSIONPLANNER=ON
            ;;
        --just_gz)
            GZ_ONLY=ON
            ;;
        --parm=*)
            PARAMFILE="${ARGI#--parm=}"
            ;;
        --parm)
            # Handle `--parm filename` syntax
            ((i++))
            PARAMFILE="${!i}"
            ;;
        --swarm=*)
            NUM_VEHICLES="${ARGI#--swarm=}"
            ;;
        --swarm)
            # Handle `--swarm num_vehicles` syntax
            ((i++))
            NUM_VEHICLES="${!i}"
            ;;
        --clean)
            echo "Cleaning up simulation files"
            # clean up simulation files except for the parm files
            rm -rf simulation/*.tlog*
            rm -rf simulation/*.ou*
            rm -rf simulation/logs
            
            echo "" > ~/moos-ivp-uav/missions/MavlinkLog.log
            exit 0
            ;;
        *)
            echo "Unknown argument: $ARGI"
            # Print help
            echo "Usage: start_simulation.sh [--gdb]: Start ArduPilot SITL in GDB mode (debugging)
                                             [--mp] : Start Mission Planner
                                             [--just_gz] : Start Gazebo only (no ArduPilot SITL)
                                             [--parm=filename | --parm filename]
                                                    :Use the specified parameter file for ArduPilot SITL
                                             [--swarm=num_vehicles | --swarm num_vehicles]
                                                    : Start Gazebo with num_vehicles vehicles
                                                      Default is 1 vehicle
                                             [--clean] : Clean up simulation files"
            exit 1
            ;;
    esac
done

# Verify that NUM_VEHICLES is a number
if ! [[ "$NUM_VEHICLES" =~ ^[0-9]+$ ]]; then
    echo "Error: --swarm value must be a number."
    exit 1
fi


PARAMFILE=$HOME/moos-ivp-uav/scripts/simulation/$PARAMFILE


mkdir -p simulation

cd ./simulation
# Start Gazebo in a new terminator window
# terminator --new-tab -e "bash -l -c 'source ~/.profile; source ~/.bashrc; sleep 2;echo \$BASH_SOURCE; env; gz sim -v4 -r skywalker_x8_runway.sdf; exec bash'" &

if [ $NUM_VEHICLES -eq 1 ]; then
    echo "Starting Gazebo with a single vehicle"
    if [ $GZ_ONLY == "ON" ]; then
        echo "Gazebo only mode. Exiting."
        gz sim -v4 -r skywalker_x8_runway.sdf 
        exit 0
    fi
    nohup gz sim -v4 -r skywalker_x8_runway.sdf &> /dev/null &
else
    echo "Starting Gazebo with $NUM_VEHICLES vehicles: SkywalkerX8_swarm$NUM_VEHICLES.world"

    if [ $GZ_ONLY == "ON" ]; then
        echo "Gazebo only mode. Exiting."
        gz sim -v4 -r SkywalkerX8_swarm$NUM_VEHICLES.world 
        exit 0
    fi
    nohup gz sim -v4 -r SkywalkerX8_swarm$NUM_VEHICLES.world &> /dev/null &
fi


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


START_PARAMS="-v ArduPlane --model JSON --add-param-file=${HOME}/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file=${PARAMFILE} -w --out=127.0.0.1:14551"

if [ $DEBUG == "ON" ]; then
    cd ${HOME}/ardupilot/ArduPlane
    echo "Starting ArduPilot SITL in GDB mode with one vehicle"
    gdb --args python ${HOME}/ardupilot/Tools/autotest/sim_vehicle.py ${START_PARAMS} #--no-mavproxy &
    exit 0
fi

# Debug is off
# Start ArduPilot SITL in normal mode
if [ $NUM_VEHICLES -eq 1 ]; then
    echo "Starting ArduPilot SITL with 1 vehicle"
    sim_vehicle.py ${START_PARAMS} #--no-mavproxy &
    # sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map --add-param-file=$PARAMFILE -w --out=127.0.0.1:14551 #--no-mavproxy &
else
    sim_vehicle.py ${START_PARAMS} --count $NUM_VEHICLES
fi


# sim_vehicle.py -v ArduPlane --model JSON  --console --map --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param -I0 --out=tcpin:0.0.0.0:8300
## + add sim params