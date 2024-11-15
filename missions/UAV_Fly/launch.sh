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

#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
VLAUNCH_ARGS="--auto --sim "
SLAUNCH_ARGS="--auto "


ARDUPILOT_IP=0.0.0.0
ARDUPILOT_PORT=14550
ARDUPILOT_PROTOCOL=udp

VNAME="skywalker"


NUM_VEHICLES=1
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
    else
        echo "$ME: Bad Arg: $ARGI. Exit Code 1."
        exit 1
    fi
done

#-------------------------------------------------------------
# Part 4: Set vnames
#-------------------------------------------------------------
# vecho "Picking vname"

# vehicle names are always deterministic in alphabetical order
# pickpos --amt=1 --vnames  > vnames.txt
# echo $VNAME > vnames.txt

# VNAMES=(`cat vnames.txt`)

VNAMES=(skywalker skyfollower)
ARDUPILOT_IPS=( "0.0.0.0" "0.0.0.0")
ARDUPILOT_PORTS=(14550 14560)
ARDUPILOT_PROTOCOLS=(udp udp)
#-------------------------------------------------------------
# Part 5: Launch the vehicles
#-------------------------------------------------------------

for INDEX in `seq 1 $NUM_VEHICLES`; do
    
    ARRAY_INDEX=$((INDEX - 1))
    VNAME=${VNAMES[$ARRAY_INDEX]}
    ARDUPILOT_IP=${ARDUPILOT_IPS[$ARRAY_INDEX]}
    ARDUPILOT_PORT=${ARDUPILOT_PORTS[$ARRAY_INDEX]}
    ARDUPILOT_PROTOCOL=${ARDUPILOT_PROTOCOLS[$ARRAY_INDEX]}

    MOOS_PORT=$(($INDEX + 9000))
    PSHARE_PORT=$(($INDEX + 9200))

    IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
    IX_VLAUNCH_ARGS+=" --vname=$VNAME"
    IX_VLAUNCH_ARGS+=" --mport=$MOOS_PORT --pshare=$PSHARE_PORT "
    IX_VLAUNCH_ARGS+=" $TIME_WARP $VERBOSE $JUST_MAKE"
    IX_VLAUNCH_ARGS+=" --ap_ip=$ARDUPILOT_IP --ap_port=$ARDUPILOT_PORT --ap_protocol=$ARDUPILOT_PROTOCOL"

    vecho "Launching: $VNAME"
    vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"

    ./launch_vehicle.sh $IX_VLAUNCH_ARGS
done
#-------------------------------------------------------------
# Part 6: Launch the Shoreside mission file
#-------------------------------------------------------------

# Join the array into a comma-separated string
VNAMES_STR=$(IFS=':'; echo "${VNAMES[*]}")

SLAUNCH_ARGS+=" $JUST_MAKE"
SLAUNCH_ARGS+=" --vnames=${VNAMES_STR[*]} "
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

