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

FORCE_LOCALHOST="no"
IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"

VNAMES="all"
USE_MOOS_SIM_PID="no"


LAT_ORIGIN=63.3975168 #-35.3632621
LON_ORIGIN=10.1435321 #149.1652374

CONFIG_FILE="./missionConfig.yaml"

XMODE=REAL

# check that ~/moos-ivp-uav-base/scripts/get_region_xy.sh exists and source it
if [ ! -f ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh ]; then
    echo "Error: File ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh not found." >&2
    exit 1
fi
source ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh




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
    echo "  --sim=,   -s                                  "
    echo "    Launch in simulation mode.                 "
    echo "  --simf , -sf ,       "
    echo "    Launch in simulation mode with localhost."
    echo "  --forcelocalhost, -fl                        " 
    echo "    Force use of localhost               " 
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
    elif [[ "${ARGI}" == --sim || "${ARGI}" == -s ]]; then
        XMODE="SIM"
        echo "Simulation mode ON."
    elif [[ "${ARGI}" == --simf || "${ARGI}" == -sf ]]; then
        XMODE="SIM"
        FORCE_LOCALHOST="yes"
        echo "Simulation mode ON with localhost."
    elif [[ "${ARGI}" == --forcelocalhost || "${ARGI}" == -fl ]]; then
        FORCE_LOCALHOST="yes"
        echo "Forcing localhost."
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
if [ "${AUTO_LAUNCHED}" = "no" -a "${IP_ADDR}" = "localhost" -a "${FORCE_LOCALHOST}" = "no" ]; then
    MAYBE_IP_ADDR=$(get_ipaddr)
    if [ $? -eq 0 ]; then
	IP_ADDR=$MAYBE_IP_ADDR
    fi
fi


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
FIRE_SPAWN_COUNT=$(get_global_val $CONFIG_FILE "missionParams.fire_spawn_count")
if [ $? -ne 0 ]; then exit 1; fi
FIRE_SPAWN_INTERVAL=$(get_global_val $CONFIG_FILE "missionParams.fire_spawn_interval")
if [ $? -ne 0 ]; then exit 1; fi


IGNORED_REGION_FILE_DEFAULT=$(get_global_val $CONFIG_FILE "missionParams.ignoredRegion_file_default")
if [ $? -ne 0 ]; then exit 1; fi
IGNORED_REGION_GENERATE=$(get_global_val $CONFIG_FILE "missionParams.ignoredRegion_generate")
if [ $? -ne 0 ]; then exit 1; fi
IGNORED_REGION_COUNT=$(get_global_val $CONFIG_FILE "missionParams.ignoredRegion_count")
if [ $? -ne 0 ]; then exit 1; fi
IGNORED_REGION_SEP_MIN=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.ignoredRegion_sep_min")
if [ $? -ne 0 ]; then exit 1; fi
IGNORED_REGION_SPAWN_COUNT=$(get_global_val $CONFIG_FILE "missionParams.ignoredRegion_spawn_count")
if [ $? -ne 0 ]; then exit 1; fi
IGNORED_REGION_SPAWN_INTERVAL=$(get_global_val $CONFIG_FILE "missionParams.ignoredRegion_spawn_interval")
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

# TMSTC* algorithm
TMSTC_GRIDSIZESENSORRANGERATIO=$(get_global_val $CONFIG_FILE "missionParams.TMSTC_gridsizeSensorRangeRatio")
if [ $? -ne 0 ]; then exit 1; fi
TMSTC_CONFIG_VMAX=$(get_global_val $CONFIG_FILE "missionParams.TMSTC_config_vmax")
if [ $? -ne 0 ]; then exit 1; fi
TMSTC_CONFIG_PHI_MAX_RAD=$(get_global_val $CONFIG_FILE "missionParams.TMSTC_config_phi_max_rad")
if [ $? -ne 0 ]; then exit 1; fi
TMSTC_POINT_FILTERING=$(get_global_val $CONFIG_FILE "missionParams.TMSTC_point_filtering")
if [ $? -ne 0 ]; then exit 1; fi

voronoi_search_enabled=$(get_global_val $CONFIG_FILE "missionParams.voronoi_search_enabled")
if [ $? -ne 0 ]; then exit 1; fi
if [ "${voronoi_search_enabled}" = "true" ]; then
    PLANNER_MODE="VORONOI_SEARCH"
else
    PLANNER_MODE="TMSTC_STAR"
fi

LOG_ENABLED=$(get_global_val $CONFIG_FILE "missionParams.log_enabled")
if [ $? -ne 0 ]; then exit 1; fi



MO_RESET_DELAY=$(get_global_val $CONFIG_FILE "missionParams.mission_operator_reset_delay")
if [ $? -ne 0 ]; then exit 1; fi
MO_VORONOI_MISSIONS=$(get_global_val $CONFIG_FILE "missionParams.mission_operator_voronoi_missions")
if [ $? -ne 0 ]; then exit 1; fi
MO_TMSTC_MISSIONS=$(get_global_val $CONFIG_FILE "missionParams.mission_operator_TMSTC_missions")
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
