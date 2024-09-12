#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh    
#  Mission: rescue_baseline
#   Author: Michael Benjamin   
#   LastEd: April 2022
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
VNAME="abe"
COLOR="yellow"
XMODE="M300"

START_POS="0,0"  
SPEED="1.2"
RETURN_POS="5,0"
MAXSPD="2"

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
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --color=<yellow>                               " 
	echo "    Color of the vehicle being launched          " 
	echo "                                                 "
	echo "  --start=<X,Y>     (default is 0,0)             " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --speed=meters/sec                             " 
	echo "    The speed use for transiting/loitering       "
	echo "  --maxspd=meters/sec                            " 
	echo "    Max speed of vehicle (for sim and in-field)  "
	echo "                                                 "
	echo "  --sim,   -s  : This is simultion not robot     "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes" 

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
	
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        XMODE="SIM"
        echo "Simulation mode ON."

    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        SPEED="${ARGI#--speed=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
        MAXSPD="${ARGI#--maxspd=*}"

    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#--------------------------------------------------------------
#  Part 3: If Heron hardware, set key info based on IP address
#--------------------------------------------------------------
if [ "${XMODE}" = "M300" ]; then
    IP_ADDR=`get_heron_info.sh --ip`
    FSEAT_IP=`get_heron_info.sh --fseat`
    VNAME=`get_heron_info.sh --name`
    if [ $? != 0 ]; then
	echo "$ME: Problem getting Heron Info. Exit Code 2"
	exit 2
    fi
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
    echo "FSEAT_IP =      [${FSEAT_IP}]     "
    echo "XMODE =         [${XMODE}]        "
    echo "----------------------------------"
    echo "START_POS =     [${START_POS}]    "
    echo "SPEED =         [${SPEED}]        "
    echo "MAXSPD =        [${MAXSPD}]       "
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
       FSEAT_IP=$FSEAT_IP           XMODE=$XMODE              \
       MAXSPD=$MAXSPD               START_POS=$START_POS      \
       COLOR=$COLOR
       
nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME \
       SPEED=$SPEED                 COLOR=$COLOR

if [ ${JUST_MAKE} = "yes" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

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
