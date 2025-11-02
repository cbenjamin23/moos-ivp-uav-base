#!/bin/bash -e
#----------------------------------------------------------
#  Script: launch_shoreside.sh
#  Author: Steve 
#  Maintainer: Charles Benjamin
#  LastEd: 06/2025
#  Description: Launches the MOOS community for the shoreside community
#-------------------------------------------------------
#  Part 1: Set convenience functions and exit actions 
#-------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi; }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

# check that ~/moos-ivp-uav/scripts/get_region_xy.sh exists and source it
if [ ! -f ~/Formula2Boat/moos-ivp-formula2boat/scripts/configfileHelperFunctions.sh ]; then
    echo "Error: File ~/Formula2Boat/moos-ivp-formula2boat/scripts/configfileHelperFunctions.sh not found." >&2
    exit 1
fi
source ~/Formula2Boat/moos-ivp-formula2boat/scripts/configfileHelperFunctions.sh

#-------------------------------------------------------
#  Part 2: Set global variables and defaults
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
VNAME="shoreside"
AUTO_LAUNCHED="no"
IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9202"
VERBOSE=""

#UAV
CMD_ARGS=""
FORCE_LOCALHOST="no"
USE_MOOS_SIM_PID="no"
LAT_ORIGIN=59.43111
LON_ORIGIN=10.46696
CONFIG_FILE="./missionConfig.yaml"
XMODE=REAL


#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_make, -j    \n" 
	printf "  --help, -h         \n" 
    echo " "
    echo " --ip=<localhost> "
    echo " Force pHostInfo to use this IP Address "
    echo " --mport=<9000> "
    echo " Port number of this vehicle's MOOSDB port "
    echo " --pshare=<9200> "
    echo " Port number of this vehicle's pShare port "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
	    JUST_MAKE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ] ; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [[ "${ARGI}" == --sim || "${ARGI}" == -s ]]; then
        XMODE="SIM"
        #USE_MOOS_SIM_PID="yes"
        echo "Simulation mode ON."
    elif [[ "${ARGI}" == --simf || "${ARGI}" == -sf ]]; then
        XMODE="SIM"
        FORCE_LOCALHOST="yes"
        echo "Simulation mode ON with localhost."
    elif [[ "${ARGI}" == --forcelocalhost || "${ARGI}" == -fl ]]; then
        FORCE_LOCALHOST="yes"
        echo "Forcing localhost."
    elif [[ "${ARGI}" == "--verbose" || "${ARGI}" == "-v" ]]; then
        VERBOSE="yes"
        
    else
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#-------------------------------------------------------
#  Part 4: UAV Config
#-------------------------------------------------------
# if [ "${AUTO_LAUNCHED}" = "no" -a "${IP_ADDR}" = "localhost" -a "${FORCE_LOCALHOST}" = "no" ]; then
#     MAYBE_IP_ADDR=$(get_ipaddr)
#     if [ $? -eq 0 ]; then
# 	IP_ADDR=$MAYBE_IP_ADDR
#     fi
# fi

# if not autolaunched 
if [ "${AUTO_LAUNCHED}" == "no" ]; then  

 
    if [ "$XMODE" == "SIM" ]; then
        TIME_WARP=$(yq eval ".simulation.time_warp" "$CONFIG_FILE")
        if [ $? -ne 0 ]; then exit 1; fi


    else # if real / field mode

        if [ ! "${FORCE_LOCALHOST}" = "yes" ]; then
            IP_ADDR=$(get_global_val $CONFIG_FILE moos.shore_ip)
            if [ $? -ne 0 ]; then exit 1; fi
        fi
        
        PSHARE_PORT=$(get_global_val $CONFIG_FILE moos.defaultPorts.PSHARE)
        if [ $? -ne 0 ]; then exit 1; fi
    
    fi

fi

# if sim

if [ "${XMODE}" = "SIM" ]; then
    USE_MOOS_SIM_PID=$(get_global_val $CONFIG_FILE simulation.useMoosSimPid)
    if [ $? -ne 0 ]; then exit 1; fi
fi



ENCOUNTER_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.encounter_radius")
if [ $? -ne 0 ]; then exit 1; fi
NEAR_MISS_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.near_miss_radius")
if [ $? -ne 0 ]; then exit 1; fi
COLLISION_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.collision_radius")
if [ $? -ne 0 ]; then exit 1; fi


MISSION_DURATION=$(get_global_val $CONFIG_FILE "missionParams.mission_duration")
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

voronoi_search_enabled=$(get_global_val $CONFIG_FILE "missionParams.voronoi_search_enabled")
if [ $? -ne 0 ]; then exit 1; fi

LOG_ENABLED=$(get_global_val $CONFIG_FILE "missionParams.log_enabled")
if [ $? -ne 0 ]; then exit 1; fi


#---------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
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
    echo "XMODE =        [${XMODE}]        "
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
    echo "TMSTC_GRIDSIZESENSORRANGERATIO = [${TMSTC_GRIDSIZESENSORRANGERATIO}]"
    echo "TMSTC_CONFIG_VMAX = [${TMSTC_CONFIG_VMAX}]"
    echo "TMSTC_CONFIG_PHI_MAX_RAD = [${TMSTC_CONFIG_PHI_MAX_RAD}]"
    echo "TMSTC_POINT_FILTERING = [${TMSTC_POINT_FILTERING}]"
    echo "----------------------------------"
    echo "PLANNER_MODE = [${PLANNER_MODE}]"
    echo "----------------------------------"
    echo "FIRE_FILE_DEFAULT = [${FIRE_FILE_DEFAULT}]"
    echo "FIRE_COLOR = [${FIRE_COLOR}]"
    echo "FIRE_GENERATE = [${FIRE_GENERATE}]"
    echo "FIRE_COUNT = [${FIRE_COUNT}]"
    echo "FIRE_SEP_MIN = [${FIRE_SEP_MIN}]"
    echo "FIRE_SPAWN_COUNT = [${FIRE_SPAWN_COUNT}]"
    echo "FIRE_SPAWN_INTERVAL = [${FIRE_SPAWN_INTERVAL}]"
    echo "----------------------------------"
    echo "IGNORED_REGION_FILE_DEFAULT = [${IGNORED_REGION_FILE_DEFAULT}]"
    echo "IGNORED_REGION_GENERATE = [${IGNORED_REGION_GENERATE}]"
    echo "IGNORED_REGION_COUNT = [${IGNORED_REGION_COUNT}]"
    echo "IGNORED_REGION_SEP_MIN = [${IGNORED_REGION_SEP_MIN}]"
    echo "IGNORED_REGION_SPAWN_COUNT = [${IGNORED_REGION_SPAWN_COUNT}]"
    echo "IGNORED_REGION_SPAWN_INTERVAL = [${IGNORED_REGION_SPAWN_INTERVAL}]"
    echo "----------------------------------"
    echo "MISSION_DURATION = [${MISSION_DURATION}]"
    echo "----------------------------------"
    echo "GRID_CELL_SIZE        = [${GRID_CELL_SIZE}]"
    echo "GRID_CELL_MAX_COUNT   = [${GRID_CELL_MAX_COUNT}]"
    echo "GRID_CELL_DECAY_TIME  = [${GRID_CELL_DECAY_TIME}]"
    echo "----------------------------------"
    echo "REGION = [${REGION}]"
    echo "----------------------------------"
    echo "LOG_ENABLED = [${LOG_ENABLED}]"
    echo "----------------------------------"
    echo "MO_RESET_DELAY = [${MO_RESET_DELAY}]"
    echo "MO_VORONOI_MISSIONS = [${MO_VORONOI_MISSIONS}]"
    echo "MO_TMSTC_MISSIONS = [${MO_TMSTC_MISSIONS}]"
    echo "=================================="
    echo -n "Hit any key to continue launch "
    read ANSWER
fi


#-------------------------------------------------------
#  Part 6: Make targ file
#-------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
NSFLAGS="--interactive --force"
fi

nsplug shoreside.moos targ_shoreside.moos $NSFLAGS WARP=$TIME_WARP \
    IP_ADDR=$IP_ADDR MOOS_PORT=$MOOS_PORT \
    PSHARE_PORT=$PSHARE_PORT LAUNCH_GUI=$LAUNCH_GUI \
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
    TMSTC_GRIDSIZESENSORRANGERATIO=$TMSTC_GRIDSIZESENSORRANGERATIO \
    TMSTC_CONFIG_VMAX=$TMSTC_CONFIG_VMAX                 \
    TMSTC_CONFIG_PHI_MAX_RAD=$TMSTC_CONFIG_PHI_MAX_RAD   \
    TMSTC_POINT_FILTERING=$TMSTC_POINT_FILTERING         \
    FIRE_FILE_DEFAULT=$FIRE_FILE_DEFAULT                 \
    FIRE_COLOR=$FIRE_COLOR                               \
    FIRE_GENERATE=$FIRE_GENERATE                         \
    FIRE_COUNT=$FIRE_COUNT                               \
    FIRE_SEP_MIN=$FIRE_SEP_MIN                           \
    FIRE_SPAWN_COUNT=$FIRE_SPAWN_COUNT                   \
    FIRE_SPAWN_INTERVAL=$FIRE_SPAWN_INTERVAL             \
    IGNORED_REGION_FILE_DEFAULT=$IGNORED_REGION_FILE_DEFAULT \
    IGNORED_REGION_GENERATE=$IGNORED_REGION_GENERATE     \
    IGNORED_REGION_COUNT=$IGNORED_REGION_COUNT           \
    IGNORED_REGION_SEP_MIN=$IGNORED_REGION_SEP_MIN       \
    IGNORED_REGION_SPAWN_COUNT=$IGNORED_REGION_SPAWN_COUNT \
    IGNORED_REGION_SPAWN_INTERVAL=$IGNORED_REGION_SPAWN_INTERVAL \
    GRID_CELL_SIZE=$GRID_CELL_SIZE                       \
    GRID_CELL_MAX_COUNT=$GRID_CELL_MAX_COUNT              \
    GRID_CELL_DECAY_TIME=$GRID_CELL_DECAY_TIME            \
    REGION=$REGION                                        \
    MISSION_DURATION=$MISSION_DURATION                    \
    XMODE=$XMODE                                          \
    PLANNER_MODE=$PLANNER_MODE                           \
    LOG_ENABLED=$LOG_ENABLED                             \
    MO_RESET_DELAY=$MO_RESET_DELAY                       \
    MO_VORONOI_MISSIONS=$MO_VORONOI_MISSIONS             \
    MO_TMSTC_MISSIONS=$MO_TMSTC_MISSIONS                 \


if [ "${JUST_MAKE}" = "yes" ]; then
echo "$ME: Targ files made; exiting without launch."
exit 0
fi

#-------------------------------------------------------
#  Part 7: Launch the processes
#-------------------------------------------------------
printf "Launching $VNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
printf "Done \n"

#----------------------------------------------------------
# Part 8: If launched from script (launch.sh), we're done, exit now
#----------------------------------------------------------

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#----------------------------------------------------------
# Part 9: Launch uMAC until the mission is quit
#----------------------------------------------------------

uMAC targ_shoreside.moos
trap SIGINT
kill -- -$$


