#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_shoreside.sh    
#  Mission: UAV_Fly
#   Author: Steve Carter Feujo Nomeny
#   LastEd: 2024       
#-------------------------------------------------------------- 
#  Part 1: Set global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"
CMD_ARGS=""
LAUNCH_GUI="yes"

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"

VNAMES=""


LAT_ORIGIN=63.3975168 #-35.3632621
LON_ORIGIN=10.1435321 #149.1652374

CONFIG_FILE="./missionConfig.yaml"


# check that ~/moos-ivp-uav/scripts/get_region_xy.sh exists and source it
if [ ! -f ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh ]; then
    echo "Error: File ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh not found." >&2
    exit 1
fi
source ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh




#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                       " 
	echo "                                                 "
	echo "Options:                                         "
        echo "  --help, -h                                     "
        echo "    Display this help message                    "
        echo "  --verbose, -v                                  "
        echo "    Increase verbosity                           "
	echo "  --just_make, -j                                " 
	echo "    Just make target files. Do not launch.       "
        echo "  --nogui, -n                                    "
        echo "    Headless mission, no pMarineViewer           "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9000>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9200>                                " 
	echo "    Port number of this vehicle's pShare port    "
    echo "  --vnames=<vnames>                            "
    echo "    Colon-separate list of all vehicle names   "
	exit 0;
    elif [[ "${ARGI//[^0-9]/}" == "$ARGI" && "$TIME_WARP" == "1" ]]; then
        TIME_WARP=$ARGI
    elif [[ "${ARGI}" == "--just_make" || "${ARGI}" == "-j" ]]; then
        JUST_MAKE="yes"
    elif [[ "${ARGI}" == "--verbose" || "${ARGI}" == "-v" ]]; then
        VERBOSE="yes"
    elif [[ "${ARGI}" == "--auto" || "${ARGI}" == "-a" ]]; then
        AUTO_LAUNCHED="yes"
    elif [[ "${ARGI}" == "--nogui" || "${ARGI}" == "-n" ]]; then
        LAUNCH_GUI="no"
    elif [[ "${ARGI}" == --ip=* ]]; then
        IP_ADDR="${ARGI#--ip=}"
    elif [[ "${ARGI}" == --mport=* ]]; then
        MOOS_PORT="${ARGI#--mport=}"
    elif [[ "${ARGI}" == --pshare=* ]]; then
        PSHARE_PORT="${ARGI#--pshare=}"
    elif [[ "${ARGI}" == --vnames=* ]]; then
        VNAMES="${ARGI#--vnames=}"
    else 
        echo "$ME: Bad Arg: $ARGI. Exit Code 1."
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: If not auto_launched (likely running in the field),
#          and the IP_ADDR has not be explicitly set, try to get
#          it using the ipaddrs.sh script. 
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "no" -a "${IP_ADDR}" = "localhost" ]; then
    MAYBE_IP_ADDR=`ipaddrs.sh --blunt`
    if [ $? = 0 ]; then
	IP_ADDR=$MAYBE_IP_ADDR
    fi
fi



USE_MOOS_SIM_PID=$(get_global_val $CONFIG_FILE simulation.useMoosSimPid)
if [ $? -ne 0 ]; then exit 1; fi


ENCOUNTER_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.encounter_radius")
if [ $? -ne 0 ]; then exit 1; fi
NEAR_MISS_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.near_miss_radius")
if [ $? -ne 0 ]; then exit 1; fi
COLLISION_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.collision_radius")
if [ $? -ne 0 ]; then exit 1; fi


SENSOR_DETECT_PD=$(get_global_val $CONFIG_FILE "missionParams.sensor_detect_pd")
if [ $? -ne 0 ]; then exit 1; fi
SENSOR_RADIUS_MAX=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.sensor_radius_max")
if [ $? -ne 0 ]; then exit 1; fi
SENSOR_RADIUS_MIN=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.sensor_radius_min")
if [ $? -ne 0 ]; then exit 1; fi
SENSOR_COLOR=$(get_global_val $CONFIG_FILE "missionParams.sensor_color")
if [ $? -ne 0 ]; then exit 1; fi
SENSOR_ALTITUDE_MAX=$(get_global_val $CONFIG_FILE "missionParams.sensor_altitude_max")
if [ $? -ne 0 ]; then exit 1; fi
SENSOR_RADIUS_FIXED=$(get_global_val $CONFIG_FILE "missionParams.sensor_radius_fixed")
if [ $? -ne 0 ]; then exit 1; fi



MISSION_DURATION=$(get_global_val $CONFIG_FILE "missionParams.mission_duration")
if [ $? -ne 0 ]; then exit 1; fi


FIRE_FILE_DEFAULT=$(get_global_val $CONFIG_FILE "missionParams.fire_file_default")
if [ $? -ne 0 ]; then exit 1; fi
FIRE_COLOR=$(get_global_val $CONFIG_FILE "missionParams.fire_color")
if [ $? -ne 0 ]; then exit 1; fi
FIRE_GENERATE=$(get_global_val $CONFIG_FILE "missionParams.fire_generate")
if [ $? -ne 0 ]; then exit 1; fi
FIRE_COUNT=$(get_global_val $CONFIG_FILE "missionParams.fire_count")
if [ $? -ne 0 ]; then exit 1; fi
FIRE_SEP_MIN=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.fire_sep_min")
if [ $? -ne 0 ]; then exit 1; fi




GRID_CELL_SIZE=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.grid_cell_size")
if [ $? -ne 0 ]; then exit 1; fi
GRID_CELL_MAX_COUNT=$(get_global_val $CONFIG_FILE "missionParams.grid_cell_max_count")
if [ $? -ne 0 ]; then exit 1; fi
GRID_CELL_DECAY_TIME=$(get_global_val $CONFIG_FILE "missionParams.grid_cell_decay_time")
if [ $? -ne 0 ]; then exit 1; fi

# read region_XY from config file
REGION=$(get_region_xy $CONFIG_FILE)
if [ $? -ne 0 ]; then exit 1; fi



#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "=================================="
    echo "  launch_shoreside.sh SUMMARY     "
    echo "=================================="
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "----------------------------------"
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "LAUNCH_GUI =    [${LAUNCH_GUI}]   "
    echo "----------------------------------"
    echo "VNAMES =        [${VNAMES}]       "
    echo "----------------------------------"
    echo "LatOrigin =     [${LAT_ORIGIN}]   "
    echo "LonOrogin =     [${LON_ORIGIN}]   "
    echo "----------------------------------"
    echo "USE_MOOS_SIM_PID = [${USE_MOOS_SIM_PID}]"
    echo "ENCOUNTER_RADIUS = [${ENCOUNTER_RADIUS}]"
    echo "NEAR_MISS_RADIUS = [${NEAR_MISS_RADIUS}]"
    echo "COLLISION_RADIUS = [${COLLISION_RADIUS}]"
    echo "----------------------------------"
    echo "SENSOR_DETECT_PD         = [${SENSOR_DETECT_PD}]"
    echo "SENSOR_RADIUS_MAX         = [${SENSOR_RADIUS_MAX}]"
    echo "SENSOR_RADIUS_MIN         = [${SENSOR_RADIUS_MIN}]"
    echo "SENSOR_COLOR          = [${SENSOR_COLOR}]"
    echo "SENSOR_ALTITUDE_MAX   = [${SENSOR_ALTITUDE_MAX}]"
    echo "SENSOR_RADIUS_FIXED   = [${SENSOR_RADIUS_FIXED}]"
    echo "----------------------------------"
    echo "FIRE_FILE_DEFAULT = [${FIRE_FILE_DEFAULT}]"
    echo "FIRE_COLOR = [${FIRE_COLOR}]"
    echo "FIRE_GENERATE = [${FIRE_GENERATE}]"
    echo "FIRE_COUNT = [${FIRE_COUNT}]"
    echo "FIRE_SEP_MIN = [${FIRE_SEP_MIN}]"
    echo "----------------------------------"
    echo "MISSION_DURATION = [${MISSION_DURATION}]"
    echo "----------------------------------"
    echo "GRID_CELL_SIZE        = [${GRID_CELL_SIZE}]"
    echo "GRID_CELL_MAX_COUNT   = [${GRID_CELL_MAX_COUNT}]"
    echo "GRID_CELL_DECAY_TIME  = [${GRID_CELL_DECAY_TIME}]"
    echo "----------------------------------"
    echo "REGION = [${REGION}]"
    echo "=================================="
    echo -n "Hit any key to continue launch "
    read ANSWER
fi

#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_shoreside.moos targ_shoreside.moos $NSFLAGS WARP=$TIME_WARP \
    IP_ADDR=$IP_ADDR       PSHARE_PORT=$PSHARE_PORT      \
    MOOS_PORT=$MOOS_PORT   LAUNCH_GUI=$LAUNCH_GUI        \
    VNAMES=$VNAMES                                       \
    LatOrigin=$LAT_ORIGIN   LonOrigin=$LON_ORIGIN        \
    USE_MOOS_SIM_PID=$USE_MOOS_SIM_PID                   \
    ENCOUNTER_RADIUS=$ENCOUNTER_RADIUS                   \
    NEAR_MISS_RADIUS=$NEAR_MISS_RADIUS                   \
    COLLISION_RADIUS=$COLLISION_RADIUS                   \
    SENSOR_DETECT_PD=$SENSOR_DETECT_PD                   \
    SENSOR_RADIUS_MAX=$SENSOR_RADIUS_MAX                 \
    SENSOR_RADIUS_MIN=$SENSOR_RADIUS_MIN                 \
    SENSOR_COLOR=$SENSOR_COLOR                           \
    SENSOR_ALTITUDE_MAX=$SENSOR_ALTITUDE_MAX             \
    SENSOR_RADIUS_FIXED=$SENSOR_RADIUS_FIXED             \
    FIRE_FILE_DEFAULT=$FIRE_FILE_DEFAULT                 \
    FIRE_COLOR=$FIRE_COLOR                               \
    FIRE_GENERATE=$FIRE_GENERATE                         \
    FIRE_COUNT=$FIRE_COUNT                               \
    FIRE_SEP_MIN=$FIRE_SEP_MIN                           \
    GRID_CELL_SIZE=$GRID_CELL_SIZE                       \
    GRID_CELL_MAX_COUNT=$GRID_CELL_MAX_COUNT              \
    GRID_CELL_DECAY_TIME=$GRID_CELL_DECAY_TIME            \
    REGION=$REGION                                        \
    MISSION_DURATION=$MISSION_DURATION


if [ ${JUST_MAKE} = "yes" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

mkdir -p Slogs
#-------------------------------------------------------
#  Part 6: Launch the processes
#-------------------------------------------------------
echo "Launching Shoreside MOOS Community. WARP="$TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
echo "Done Launching Shoreside Community"


#-------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#-------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#-------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#-------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$
