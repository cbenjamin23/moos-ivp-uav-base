#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh    
#  Mission: UAV_Fly
#   Author: Steve Carter Feujo Nomeny   
#   LastEd: 2024
#--------------------------------------------------------------
#  Part 1: Declare global var defaults
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE="no"
AUTO_LAUNCHED="no"
CMD_ARGS=""


IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"

SHORE_IP="localhost"
SHORE_PSHARE="9200"
VNAME="skywalker"
VIDX=-1 # Default vehicle index
COLOR="yellow"
XMODE="REAL"

START_POS="50,50"  
SPEED="15"
RETURN_POS="0,0"
MAXSPD="30"


LAT_ORIGIN=63.3975168 #-35.3632621 
LON_ORIGIN=10.1435321 #149.1652374 

# Ardupilot SITL
ARDUPILOT_IP=0.0.0.0
ARDUPILOT_PORT=14550
ARDUPILOT_PROTOCOL=udp


CONFIG_FILE="./missionConfig.yaml"
USE_MOOS_SIM_PID="no"


#--------------------------------------------------------------
#  Functions
#--------------------------------------------------------------

# check that ~/moos-ivp-uav/scripts/get_region_xy.sh exists and source it
if [ ! -f ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh ]; then
    echo "Error: File ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh not found." >&2
    exit 1
fi
source ~/moos-ivp-uav/scripts/configfileHelperFunctions.sh



#--------------------------------------------------------------
# Get all config variables
#--------------------------------------------------------------




MAXSPD=$(get_global_val $CONFIG_FILE field.speed.max)
if [ $? -ne 0 ]; then exit 1; fi
MINSPD=$(get_global_val $CONFIG_FILE field.speed.min)
if [ $? -ne 0 ]; then exit 1; fi

# Define the interval in steps between the differen speeds
SPD_STEPS=$(($MAXSPD - $MINSPD + 1))



CAPTURE_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.capture_radius")
if [ $? -ne 0 ]; then exit 1; fi


SLIP_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.slip_radius")
if [ $? -ne 0 ]; then exit 1; fi

ENCOUNTER_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.encounter_radius")
if [ $? -ne 0 ]; then exit 1; fi

REGION=$(get_region_xy $CONFIG_FILE)
if [ $? -ne 0 ]; then exit 1; fi



#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "$ME [OPTIONS] [time_warp]                        "
	echo "                                                 " 
	echo "Options:                                         "
	echo "  --help, -h                                     " 
	echo "    Print this help message and exit             "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching     "
    echo "  --auto, -a                                     "
    echo "     Auto-launched by a script.                  "
    echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9001>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9201>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --shore=<localhost>                            " 
	echo "    IP address location of shoreside             "
	echo "  --shore_pshare=<9200>                          " 
	echo "    Port on which shoreside pShare is listening  "
	echo "  --vname=<skywalker>                            " 
	echo "    Name of the vehicle being launched           " 
	echo "  --color=<yellow>                               " 
	echo "    Color of the vehicle being launched          " 
	echo "                                                 "
    echo "  --ap_ip=<0.0.0.0>                     "
    echo "    IP of the ArduPilot autopilot                " 
    echo "    Device  for coms with ArduPilot autopilot  <ttySAC0>   " 
    echo "  --ap_port=<14550>                       "
    echo "    Port of the ArduPilot autopilot              "
    echo "    Baudrate for coms with ArduPilot autopilot   "
    echo "  --ap_protocol=<udp>                       "
    echo "    Protocol for coms with ArduPilot autopilot   "
    echo "    udp, tcp or serial                           "
    echo "                                                 "
	echo "  --start=<X,Y>     (default is 0,0)             " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --speed=meters/sec                             " 
	echo "    The speed use for transiting/loitering       "
	echo "  --maxspd=meters/sec                            " 
	echo "    Max speed of vehicle (for sim and in-field)  "
    echo "  --return=<X,Y>     (default is 0,0)            "
    echo "    Return position chosen by script launching   "
	echo "                                                 "
	echo "  --sim,   -s  : This is simultion not robot     "
    echo "  --id=<0>     : Index of the vehicle      "
	exit 0;
    elif [[ "${ARGI//[^0-9]/}" == "$ARGI" && "$TIME_WARP" == "1" ]]; then
        TIME_WARP=$ARGI
    elif [[ "${ARGI}" == "--just_make" || "${ARGI}" == "-j" ]]; then
        JUST_MAKE="yes"
    elif [[ "${ARGI}" == "--verbose" || "${ARGI}" == "-v" ]]; then
        VERBOSE="yes"
    elif [[ "${ARGI}" == "--auto" || "${ARGI}" == "-a" ]]; then
        AUTO_LAUNCHED="yes"
    elif [[ "${ARGI}" == --ip=* ]]; then
        IP_ADDR="${ARGI#--ip=}"
    elif [[ "${ARGI}" == --mport=* ]]; then
        MOOS_PORT="${ARGI#--mport=}"
    elif [[ "${ARGI}" == --pshare=* ]]; then
        PSHARE_PORT="${ARGI#--pshare=}"
    elif [[ "${ARGI}" == --shore=* ]]; then
        SHORE_IP="${ARGI#--shore=}"
    elif [[ "${ARGI}" == --shore_pshare=* ]]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=}"
    elif [[ "${ARGI}" == --vname=* ]]; then
        VNAME="${ARGI#--vname=}"
    elif [[ "${ARGI}" == --color=* ]]; then
        COLOR="${ARGI#--color=}"
    elif [[ "${ARGI}" == "--sim" || "${ARGI}" == "-s" ]]; then
        XMODE="SIM"
        echo "Simulation mode ON."
    elif [[ "${ARGI}" == --start=* ]]; then
        START_POS="${ARGI#--start=}"
    elif [[ "${ARGI}" == --return=* ]]; then
        RETURN_POS="${ARGI#--return=}"
    elif [[ "${ARGI}" == --speed=* ]]; then
        SPEED="${ARGI#--speed=}"
    elif [[ "${ARGI}" == --maxspd=* ]]; then
        MAXSPD="${ARGI#--maxspd=}"
    elif [[ "${ARGI}" == --ap_ip=* ]]; then
        ARDUPILOT_IP="${ARGI#--ap_ip=}"
    elif [[ "${ARGI}" == --ap_port=* ]]; then
        ARDUPILOT_PORT="${ARGI#--ap_port=}"
    elif [[ "${ARGI}" == --ap_protocol=* ]]; then
        ARDUPILOT_PROTOCOL="${ARGI#--ap_protocol=}"
    elif [[ "${ARGI}" == --id=* ]]; then
        VIDX="${ARGI#--id=}"
        echo "Vehicle ID set to: ${VIDX}"
    else
        echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
        exit 1
    fi
done

# #--------------------------------------------------------------
# #  Part 3: Configure Variables 
# #--------------------------------------------------------------


# if not autolaunched 
if [ "${AUTO_LAUNCHED}" == "no" ]; then  


    if [ -n "$WHICH_ODROID" ]; then
        echo "Odroid detected. Setting Vehicle Nr to $WHICH_ODROID"
        VIDX=$WHICH_ODROID
    fi

    if [ $VIDX -eq -1 ]; then
        echo "Error: Vehicle index not set. Exit Code 1."
        echo " Set using --id=<0>"
        exit 1
    fi

    VNAME=$(yq eval ".drones[$VIDX].name" "$CONFIG_FILE")

    COLOR=$(get_val_by_drone_name $CONFIG_FILE "$VNAME" "color")
    if [ $? -ne 0 ]; then exit 1; fi

    x=$(get_val_by_drone_name $CONFIG_FILE "$VNAME" "start_orientaton_moos.x")
    if [ $? -ne 0 ]; then exit 1; fi
    y=$(get_val_by_drone_name $CONFIG_FILE "$VNAME" "start_orientaton_moos.y")
    if [ $? -ne 0 ]; then exit 1; fi

    ## Convert [m] to moos distance (1m = 2 moos distance)
    x=$(($x * 2))
    y=$(($y * 2))    
    START_POS="$x,$y"

    if [ "$XMODE" == "SIM" ]; then
        MOOS_PORT=$((MOOS_PORT + VIDX ))
        PSHARE_PORT=$((PSHARE_PORT + VIDX))
        TIME_WARP=$(yq eval ".simulation.time_warp" "$CONFIG_FILE")
        if [ $? -ne 0 ]; then exit 1; fi


        USE_MOOS_SIM_PID=$(get_global_val $CONFIG_FILE simulation.useMoosSimPid)
        if [ $? -ne 0 ]; then exit 1; fi
    
    else # if real / field mode

        IP_ADDR=$(get_ipaddr)
        if [ $? -ne 0 ]; then exit 1; fi


        SHORE_IP=$(get_global_val $CONFIG_FILE moos.shore_ip)
        if [ $? -ne 0 ]; then exit 1; fi
        SHORE_PSHARE=$(get_global_val $CONFIG_FILE moos.defaultPorts.PSHARE)
        if [ $? -ne 0 ]; then exit 1; fi
    
    fi

fi

#If Skywalker hardware, set ArduPilot access info to known values
if [ "${XMODE}" = "REAL" ]; then
    ARDUPILOT_IP=ttySAC0
    ARDUPILOT_PORT=115200
    ARDUPILOT_PROTOCOL=serial

else # if simulation

    SPEED=15 # It is 15 in simulation


fi




#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "VNAME =         [${VNAME}]        "
    echo "COLOR =         [${COLOR}]        "
    echo "----------------------------------"
    echo "XMODE =         [${XMODE}]        "
    echo "----------------------------------"
    echo "START_POS =     [${START_POS}]    "
    echo "SPEED =         [${SPEED}]        "
    echo "MAXSPD =        [${MAXSPD}]       "
    echo "MINSPD =        [${MINSPD}]       "
    echo "SPD_STEPS =     [${SPD_STEPS}]    "
    echo "----------------------------------"
    echo "LatOrigin =     [${LAT_ORIGIN}]    "
    echo "LonOrogin =     [${LON_ORIGIN}]    "
    echo "----------------------------------"
    echo "USE_MOOS_SIM_PID = [${USE_MOOS_SIM_PID}]"
    echo "CAPTURE_RADIUS = [${CAPTURE_RADIUS}]"
    echo "SLIP_RADIUS =    [${SLIP_RADIUS}]"
    echo "REGION =       [${REGION}]"
    echo "----------------------------------"
    echo "ARDUPILOT_IP =  [${ARDUPILOT_IP}]  "
    echo "ARDUPILOT_PORT =[${ARDUPILOT_PORT}]"
    echo "ARDUPILOT_PROTOCOL =[${ARDUPILOT_PROTOCOL}]"
    echo "----------------------------------"
    echo "V_INDEX          =    [${VIDX}]   "
    echo "----------------------------------"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi


#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME              \
       IP_ADDR=$IP_ADDR             SHORE_IP=$SHORE_IP        \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT      \
       COLOR=$COLOR                 XMODE=$XMODE                          \
       LatOrigin=$LAT_ORIGIN        LonOrigin=$LON_ORIGIN     \
       AP_IP=$ARDUPILOT_IP          AP_PORT=$ARDUPILOT_PORT   \
       AP_PROTOCOL=$ARDUPILOT_PROTOCOL      START_POS=$START_POS \
       MIN_SPEED=$MINSPD  MAX_SPEED=$MAXSPD SPEED_STEPS=$SPD_STEPS  \
       USE_MOOS_SIM_PID=$USE_MOOS_SIM_PID                      \
       REGION=$REGION

nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME \
       SPEED=$SPEED                  START_POS=$START_POS     \
       LatOrigin=$LAT_ORIGIN         LonOrogin=$LON_ORIGIN    \
       XMODE=$XMODE                  COLOR=$COLOR             \
       USE_MOOS_SIM_PID=$USE_MOOS_SIM_PID                     \
       CAPTURE_RADIUS=$CAPTURE_RADIUS SLIP_RADIUS=$SLIP_RADIUS \
         
                                                
       
if [ ${JUST_MAKE} = "yes" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi


mkdir -p Vlogs
#-------------------------------------------------------
#  Part 6: Launch the vehicle mission
#-------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
echo "Done Launching $VNAME MOOS Community"

#---------------------------------------------------------------
#  Part 8: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 9: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
