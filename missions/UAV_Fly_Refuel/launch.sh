#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch.sh    
#   Author: Steve Carter Feujo Nomeny   
#   LastEd: 2024
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }



# check that ~/moos-ivp-uav-base/scripts/get_region_xy.sh exists and source it
if [ ! -f ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh ]; then
    echo "Error: File ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh not found." >&2
    exit 1
fi
source ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh


#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
VLAUNCH_ARGS="--auto --sim "
SLAUNCH_ARGS="--auto --sim"


ARDUPILOT_IP=0.0.0.0
ARDUPILOT_PORT=14550
ARDUPILOT_PROTOCOL=udp

VNAME="skywalker"

START_FROM_CONFIG="yes"  # default is to start from config file
CONFIG_FILE="./missionConfig.yaml"

NUM_VEHICLES=1


#### Defaults 
VNAMES=(skywalker skyfollower)
ARDUPILOT_IPS=( "0.0.0.0" "0.0.0.0")
ARDUPILOT_PORTS=(14550 14560)
ARDUPILOT_PROTOCOLS=(udp udp)
COLOR="blue"

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [[ "${ARGI}" == "--help" || "${ARGI}" == "-h" ]]; then
	echo "$ME: [OPTIONS] [time_warp]                          "
	echo "                                                    "
	echo "Options:                                            "
        echo "  --help, -h                                        "
        echo "    Display this help message                       "
        echo "  --verbose, -v                                     "
        echo "    Increase verbosity                              "
	echo "  --just_make, -j                                   " 
	echo "    Just make the targ files, but do not launch.    " 
    echo "  --vname=<skywalker>                                  " 
	echo "    Name of the vehicle being launched           " 
    echo "  --ap_ip=<0.0.0.0>                     "
    echo "    IP of the ArduPilot autopilot                " 
    echo "    Device  for coms with ArduPilot autopilot  <ttySAC0>   " 
    echo "  --ap_port=<14550>                       "
    echo "    Port of the ArduPilot autopilot              "
    echo "    Baudrate for coms with ArduPilot autopilot   "
    echo "  --ap_protocol=<udp>                       "
    echo "    Protocol for coms with ArduPilot autopilot   "
    echo "    udp, tcp or serial                           "
    echo "  --swarm=<1>                       "
    echo "    Number of vehicles in the swarm              "
    echo " --config, -c=<missionConfig.yaml>                       "
    echo "    Start the vehicles from a configuration file  "
	exit 0;
    elif [[ "${ARGI}" == "--verbose" || "${ARGI}" == "-v" ]]; then
        VERBOSE="--verbose"
    elif [[ "${ARGI//[^0-9]/}" == "$ARGI" && "$TIME_WARP" == "1" ]]; then 
        TIME_WARP=$ARGI
    elif [[ "${ARGI}" == "--just_make" || "${ARGI}" == "-j" ]]; then
        JUST_MAKE="-j"
    elif [[ "${ARGI}" == --vname=* ]]; then
        VNAME="${ARGI#--vname=}"
    elif [[ "${ARGI}" == --ap_ip=* ]]; then
        ARDUPILOT_IP="${ARGI#--ap_ip=}"
    elif [[ "${ARGI}" == --ap_port=* ]]; then
        ARDUPILOT_PORT="${ARGI#--ap_port=}"
    elif [[ "${ARGI}" == --ap_protocol=* ]]; then
        ARDUPILOT_PROTOCOL="${ARGI#--ap_protocol=}"
    elif [[ "${ARGI}" == --swarm=* ]]; then
        NUM_VEHICLES="${ARGI#--swarm=}"
    elif [[ "${ARGI}" == --config=* || "${ARGI}" == -c=* ]]; then
        MISSION_CONFIG="${ARGI#--config=}"
        if [ -f $MISSION_CONFIG ]; then # Check if the file exists
            CONFIG_FILE=$MISSION_CONFIG
        else
        echo "Configuration file not found: $MISSION_CONFIG. 
              Using default config fileExit Code 1."
        fi
        START_FROM_CONFIG="yes"
    elif [[ "${ARGI}" == "--config" || "${ARGI}" == "-c" ]]; then
        START_FROM_CONFIG="yes"
    else
        echo "Bad Arg: $ARGI. Exit Code 1."
        exit 1
    fi
done

#-------------------------------------------------------------
# Part 4: Set vnames
#-------------------------------------------------------------
# vecho "Picking vname"

if [ "$START_FROM_CONFIG" == "yes" ]; then
    vecho "Starting vehicles from configuration file: $CONFIG_FILE"
    vecho "All other arguments will be ignored"

    NUM_VEHICLES=$(yq eval '.simulation.number_of_drones' "$CONFIG_FILE")
    NUM_SCOUT_DRONES=$(yq eval '.simulation.num_scout_drones' "$CONFIG_FILE")
    ARDUPILOT_IP=$(yq eval ".simulation.ardupilot_ip" "$CONFIG_FILE")
    ARDUPILOT_PROTOCOL=$(yq eval ".simulation.ardupilot_protocol" "$CONFIG_FILE")

    DEFAULT_PORT_DB=$(yq eval ".moos.defaultPorts.DB" "$CONFIG_FILE")
    DEFAULT_PORT_PSHARE=$(yq eval ".moos.defaultPorts.PSHARE" "$CONFIG_FILE")
    VNAMES=""

    TIME_WARP=$(yq eval ".simulation.time_warp" "$CONFIG_FILE")

else
    vecho "Starting vehicles from command line arguments"
    VNAMES=(skywalker skyfollower)
    ARDUPILOT_IPS=( "0.0.0.0" "0.0.0.0")
    ARDUPILOT_PORTS=(14550 14560)
    ARDUPILOT_PROTOCOLS=(udp udp)
fi


# vehicle names are always deterministic in alphabetical order
# pickpos --amt=1 --vnames  > vnames.txt
# echo $VNAME > vnames.txt

# VNAMES=(`cat vnames.txt`)


#-------------------------------------------------------------
# Part 5: Launch the vehicles
#-------------------------------------------------------------

for ((i = 0; i < $NUM_VEHICLES; i++)); do

    # Mark scouts as first NUM_SCOUT_DRONES indices
    if (( i < NUM_SCOUT_DRONES )); then
        DEPLOY_AT_START="true"
    else
        DEPLOY_AT_START="false"
    fi
    
    if [ "$START_FROM_CONFIG" == "yes" ]; then
        VNAME=$(yq eval ".drones[$i].name" "$CONFIG_FILE")
        ARDUPILOT_IP=$(yq eval ".simulation.ardupilot_ip" "$CONFIG_FILE")
        ARDUPILOT_PORT=$(yq eval ".simulation.ardupilot_port_default" "$CONFIG_FILE")
        
        
        x=$(get_val_by_drone_name $CONFIG_FILE "$VNAME" "start_orientaton_moos.x")
        if [ $? -ne 0 ]; then exit 1; fi
        y=$(get_val_by_drone_name $CONFIG_FILE "$VNAME" "start_orientaton_moos.y")
        if [ $? -ne 0 ]; then exit 1; fi

        ## Convert [m] to moos distance (1m = 2 moos distance)
        x=$(($x * 2))
        y=$(($y * 2))

        ARDUPILOT_PORT=$(($ARDUPILOT_PORT + $i*10))
    
        START_POS="$x,$y"
        
        COLOR=$(yq eval ".drones[$i].color" "$CONFIG_FILE")
        if [ $? -ne 0 ]; then exit 1; fi

        MOOS_PORT=$(($i+1 + $DEFAULT_PORT_DB))
        PSHARE_PORT=$(($i+1 + $DEFAULT_PORT_PSHARE))

        VNAMES="${VNAMES:+$VNAMES:}$VNAME" # append name to VNAMES
        
    else
        VNAME=${VNAMES[$i]}
        ARDUPILOT_IP=${ARDUPILOT_IPS[$i]}
        ARDUPILOT_PORT=${ARDUPILOT_PORTS[$i]}
        ARDUPILOT_PROTOCOL=${ARDUPILOT_PROTOCOLS[$i]}

        MOOS_PORT=$(($i+1 + 9000))
        PSHARE_PORT=$(($i+1 + 9200))
    fi

    IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
    IX_VLAUNCH_ARGS+=" --vname=$VNAME"
    IX_VLAUNCH_ARGS+=" --mport=$MOOS_PORT --pshare=$PSHARE_PORT "
    IX_VLAUNCH_ARGS+=" $TIME_WARP $VERBOSE $JUST_MAKE"
    IX_VLAUNCH_ARGS+=" --color=$COLOR"
    IX_VLAUNCH_ARGS+=" --start=$START_POS "
    IX_VLAUNCH_ARGS+=" --ap_ip=$ARDUPILOT_IP --ap_port=$ARDUPILOT_PORT --ap_protocol=$ARDUPILOT_PROTOCOL"
    IX_VLAUNCH_ARGS+=" --deploy_at_start=$DEPLOY_AT_START"

    vecho "Launching: $VNAME"
    vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"

    ./launch_vehicle.sh $IX_VLAUNCH_ARGS
done
#-------------------------------------------------------------
# Part 6: Launch the Shoreside mission file
#-------------------------------------------------------------

if [ "$START_FROM_CONFIG" == "no" ]; then
    # Join the array into a comma-separated string
    VNAMES=$(IFS=':'; echo "${VNAMES[*]}")
    VNAMES="${VNAMES}[*]"
fi


SLAUNCH_ARGS+=" $JUST_MAKE"
SLAUNCH_ARGS+=" --vnames=${VNAMES} "
vecho "Launching the shoreside. Args: $SLAUNCH_ARGS $TIME_WARP"

./launch_shoreside.sh $SLAUNCH_ARGS $VERBOSE $TIME_WARP 

# if JUST_MAKE is set, then we are done
if [ -n "${JUST_MAKE}" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------------
# Part 7: Launch uMac until the mission is quit
#-------------------------------------------------------------
uMAC targ_shoreside.moos

kill -- -$$

