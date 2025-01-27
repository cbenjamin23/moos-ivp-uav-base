#!/bin/bash
ME=`basename "$0"`
VERBOSE=OFF
DEBUG=OFF
MISSIONPLANNER=OFF
PARAMFILE=sim.parm

NUM_VEHICLES=1

NEW_TERMINAL="NO"

GZ_ONLY=OFF

READ_CONFIG=NO

CONFIG_FILE="$HOME/moos-ivp-uav/missions/UAV_Fly/missionConfig.yaml"
FORCE=""

vecho() { if [ "$VERBOSE" != "" ]; then echo " -> $ME: $1"; fi }

kill_drone_session () {
    if tmux has-session -t drone_sim 2>/dev/null; then
        echo "Terminating existing tmux session: drone_sim"
        
        local i=0
        for pane in $(tmux list-panes -t drone_sim -F '#{pane_id}'); do
            tmux send-keys -t "$pane" C-c
            i=$(($i+1))
        done

        local time=3 #max 3 second sleed
        if [ $i -lt $time ]; then time=$i; fi
        
        sleep $time # Wait till all programs have been terminated
        
        tmux kill-session -t drone_sim
        vecho "tmux session drone_sim terminated."
    fi
}

cleanup() {
    for file in ~/moos-ivp-uav/missions/MavlinkLog*.log; do
        > "$file"
    done

    excluded_files=("sim.parm" "hardware.parm" "mav_sim_old.parm" "way.txt")

    exclusions=()
    for file in "${excluded_files[@]}"; do
        exclusions+=(! -name "$file")
    done

    files_to_delete=$(find simulation -type f "${exclusions[@]}" -print)
    if [ -z "$files_to_delete" ]; then
        echo "No files to delete"
        exit 0
    fi

    echo "Files to be deleted:"
    echo "$files_to_delete"
    
    #confirm deletion by user
    read -p "Do you want to delete these files? (y/n) " -n 1 -r
    echo

    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        find "simulation" -type f "${exclusions[@]}" -delete
        find "simulation" -type d -empty -delete
        echo "Cleaned up simulation files"
    else
        echo "Aborting deletions"
    fi
    exit 0
}

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
            # clean up simulation files except for the parm files
            cleanup
            exit 0
            ;;
        --config)
            echo "Usig configuration file"
            READ_CONFIG="YES"
            ;;
        -c)
            echo "Usig configuration file"
            READ_CONFIG="YES"
            ;;
        --force)
            FORCE='--forceOverwrite'
            ;;
        -f)
            FORCE='--forceOverwrite'
            ;;

        -v)
            VERBOSE=ON
            ;;
        --nt)
            NEW_TERMINAL="YES"
            ;;
        --ks)
            echo "Killing existing tmux session drone_sim if exists and exiting."
            kill_drone_session
            exit 0
            ;;
        --verbose)
            VERBOSE=ON
            ;; 
        *)
            echo "Unknown argument: $ARGI"
            # Print help
            echo "Usage: start_simulation.sh 
        [--gdb]: Start ArduPilot SITL in GDB mode (debugging)
        [--mp] : Start Mission Planner
        [--just_gz] : Start Gazebo only (no ArduPilot SITL)
        [--parm=filename | --parm filename]
            :Use the specified parameter file for ArduPilot SITL
        [--swarm=num_vehicles | --swarm num_vehicles]
            : Start Gazebo with num_vehicles vehicles
              Default is 1 vehicle
        [--clean] : Clean up simulation files
        [--config] : Use the configuration file to generate drone SDF files
        [--force | -f] : Force overwrite of existing drone SDF files and restart Gazebo
        [--verbose | -v] : Verbose output
        [--nt] : Open new terminal windows for the simulation
        [--ks] : Kill existing tmux session and exit"

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



# num vehicles greater than 1
if [ $READ_CONFIG == "YES" ]; then
    echo "Generating SDF files from configuration file"
    python /home/steve/moos-ivp-uav/scripts/generate_drone_sdf.py $FORCE\
        --config_file               $CONFIG_FILE \
        --default_models_folder     ~/SITL_Models/Gazebo/models/skywalker_x8/ \
        --destination_models_folder ~/moos-ivp-uav/GazeboSim/models/ \
        --destination_worlds_folder ~/moos-ivp-uav/GazeboSim/worlds/  \
        $FORCE
    
    
    NUM_VEHICLES=$(yq eval '.simulation.number_of_drones' "$CONFIG_FILE")
    vecho "Number of vehicles: $NUM_VEHICLES"
fi






# Start Gazebo in a new terminator window
# terminator --new-tab -e "bash -l -c 'source ~/.profile; source ~/.bashrc; sleep 2;echo \$BASH_SOURCE; env; gz sim -v4 -r skywalker_x8_runway.sdf; exec bash'" &

run_gazebo() {
    # Check if Gazebo is running
    local num_vehicles=$1
    local gz_pids=$(pgrep -f "gz sim")
    
    if [ -n "$gz_pids" ]; then
        vecho "Gazebo is already running"
        
        if [ "$FORCE" == "" ]; then
            echo "Gazebo is already running. Use --force to restart."
            return 0
        fi

        echo "Force flag is set. Restarting Gazebo..."
        echo "$gz_pids" | xargs kill -9
        kill -9 $gz_pids
        sleep 2 # Give it some time to clean up
    fi

    if [ $num_vehicles -gt 0 ]; then
        echo "Starting Gazebo with $num_vehicles vehicles: SkywalkerX8_swarm$num_vehicles.world"
        if [ $GZ_ONLY == "ON" ]; then
            echo "Gazebo only mode. Exiting."
            gz sim -v4 -r SkywalkerX8_swarm$num_vehicles.world 
            exit 0
        fi
        nohup gz sim -v4 -r SkywalkerX8_swarm$num_vehicles.world &> /dev/null &
    fi
    vecho "Gazebo started."
}


mkdir -p simulation
cd ./simulation


# Start Gazebo
run_gazebo $NUM_VEHICLES



if [ $MISSIONPLANNER == "ON" ]; then
    # Start Mission Planner in the background in a third terminator window
    echo "Starting Mission Planner"
    # Start in new xterm window
    xterm -e "bash -c 'source ~/.profile; cd ~/MissionPlanner; mono MissionPlanner.exe; exec bash'" &
fi



START_PARAMS="-v ArduPlane --model JSON --add-param-file=${HOME}/SITL_Models/Gazebo/config/skywalker_x8.param --add-param-file=${PARAMFILE} -w"
START_PARAMS_FULL="$START_PARAMS --out=127.0.0.1:14551 --console --map"

if [ $DEBUG == "ON" ]; then
    cd ${HOME}/ardupilot/ArduPlane
    echo "Starting ArduPilot SITL in GDB mode with 1 vehicle"
    gdb --args python ${HOME}/ardupilot/Tools/autotest/sim_vehicle.py ${START_PARAMS} #--no-mavproxy &
    exit 0
fi

### FUNTIONS

calculate_lat_lon_from_XYoffsett() {
    local orig_lat=$1  # Original latitude
    local orig_lon=$2  # Original longitude
    local offset_x=$3  # Offset in meters (east/west)
    local offset_y=$4  # Offset in meters (north/south)

    # Earth's radius [m]
    local earth_radius=6378137
    local pi=3.14159265359

    local start_lat=$(echo "$orig_lat + ($offset_y / $earth_radius) * (180 / $pi)" | bc -l)
    local start_lon=$(echo "$orig_lon + ($offset_x / ($earth_radius * c($orig_lat * ($pi / 180)))) * (180 / $pi)" | bc -l)

    echo "$start_lat,$start_lon"
}


simulate_vehicles() {
    # Fill out here
    # The function should start n vehicles in the simulation by opening a new tab and running the sim_vehicle.py script
    # swarm simulation needs a --sysid (from config) and -I{id-1} to tell the ardupilot instance to incrament it's UPD connection use

    echo "Starting ArduPilot SITL with ${NUM_VEHICLES} vehicle(s)"
    
    local num_vehicles=$1

    kill_drone_session

    local time_warp=$(yq eval '.simulation.time_warp' "$CONFIG_FILE")
    local home_altitude=$(yq eval '.simulation.home_altitude' "$CONFIG_FILE")
    local datum_lat=$(yq eval '.moos.datum.lat' "$CONFIG_FILE")
    local datum_lon=$(yq eval '.moos.datum.lon' "$CONFIG_FILE")

    vecho "Time Warp: $time_warp"
    vecho "Home Altitude: $home_altitude"
    vecho "Datum Lat: $datum_lat"
    vecho "Datum Lon: $datum_lon"


    # Open a new terminal with split panes for vehicles
    # Detach and name it drone_sim
    tmux new-session -d -s drone_sim "echo 'Launching $num_vehicles drones'; bash"

    # Loop through the drones
    for ((i = 0; i < num_vehicles; i++)); do
        

        local drone_id=$((i + 1))
        local drone_name=$(yq eval ".drones[$i].name" "$CONFIG_FILE")
        local drone_start_x=$(yq eval ".drones[$i].start_orientaton_moos.x" "$CONFIG_FILE")
        local drone_start_y=$(yq eval ".drones[$i].start_orientaton_moos.y" "$CONFIG_FILE")
        local drone_start_hdg=$(yq eval ".drones[$i].start_orientaton_moos.hdg" "$CONFIG_FILE")

        vecho "Drone Instance: $i"
        vecho "Drone id: $drone_id"
        vecho "Drone name: $drone_name"
        vecho "Drone start x: $drone_start_x"
        vecho "Drone start y: $drone_start_y"
        vecho "Drone start hdg: $drone_start_hdg"

        local start_lat_lon=$(calculate_lat_lon_from_XYoffsett $datum_lat $datum_lon $drone_start_x $drone_start_y)
        vecho "Start Lat/Lon: $start_lat_lon"
        
        local missionplanner_outport=$((14551 + $i * 10)) #UDP localhost
        ## IMPORTANT that SYSID_THISMAV and SIM_SPEEDUP are not set in the param file
        local args="$START_PARAMS --console -I${i}"
        args+=" --out=127.0.0.1:${missionplanner_outport}"
        args+=" --sysid=$drone_id"
        args+=" --speedup=$time_warp"
        args+=" --custom-location=$start_lat_lon,$home_altitude,$drone_start_hdg"

        if [ $i -eq 0 ]; then
            args+=" --map"
        fi

        local dirname="${drone_name}_logs"
        
        # Split tmux pane for each vehicle and run the command
        if (( i > 0 )); then
            tmux split-window -h
        fi
        tmux send-keys "mkdir -p $dirname; cd $dirname; source ~/.bashrc; sim_vehicle.py ${args}" C-m
      
        
        vecho "Started with command: sim_vehicle.py ${args}"

        sleep 1 # wait the first vehicle to start before starting the next one
    done

    # Attach to the tmux session for the user to monitor
    tmux select-layout tiled
    
    if [ $NEW_TERMINAL == "YES" ]; then
        # terminator -e "tmux attach-session -t drone_sim; bash" &
        terminator -e "tmux attach-session -t drone_sim; bash" &
        return
    fi

    echo "Attaching to tmux session: drone_sim in 3 seconds"
    sleep 3
    tmux attach-session -t drone_sim
    
}




if [ $READ_CONFIG == "NO" ]; then
    echo "Starting ArduPilot SITL with $NUM_VEHICLES vehicles"
    sim_vehicle.py ${START_PARAMS_FULL} --sysid=1 --speedup=1 ## IMPORTANT that SYSID_THISMAV and SIM_SPEEDUP are not set in the param file
    exit 0
fi

simulate_vehicles $NUM_VEHICLES

