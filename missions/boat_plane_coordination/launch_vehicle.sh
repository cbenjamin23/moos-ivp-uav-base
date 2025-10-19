#!/bin/bash -e
#----------------------------------------------------------
#  Script: launch_vehicle.sh
#  Author: Steve 
#  Maintainer: Charles Benjamin
#  LastEd: 06/2025
#  Description: Launches the MOOS community for the vehicle and shorside communities
#-------------------------------------------------------
#  Part 1: Set convenience functions and exit actions
#----------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi; }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT 

# --------------------------------------------------------
# Part 2: Set global variables and defaults
#--------------------------------------------------------

TIME_WARP=1
COMMUNITY="formula2boat"
GUI="yes"
ONBOARD="no"
AUTO_LAUNCHED="no"
IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"
SHORE_IP="localhost"
SHORE_PSHARE="9202"
VNAME="formula2boat"
BOLLARD="no"
VERBOSE="no"


#-------------------------  ---------------------------------
#  Part 3: Check for and handle command-line arguments
#----------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch.sh [SWITCHES] [time_warp]   "
	echo "  --help, -h           Show this help message            " 
    echo " "
    echo " --ip=<localhost> Veh Default IP addr "
    echo " --mport=<9001> Veh MOOSDB port "
    echo " --pshare=<9201> Veh pShare listen port "
    echo " --shore=<localhost> Shoreside IP to try "
    echo " --shore_pshare=<9200> Shoreside pShare port "

	exit 0;
    elif [ "${ARGI}" = "--nogui" ] ; then
	    GUI="no"
    elif [ "${ARGI}" = "--onboard" ] ; then
	    ONBOARD="yes"
    elif [ "${ARGI}" = "-v" ] ; then
	    VERBOSE="yes"
    elif [ "${ARGI}" = "--bollard" ] ; then
        BOLLARD="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ] ; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then
    echo "=========================================="
    echo "         launch_vehicle.sh SUMMARY         "
    echo "=========================================="
    echo "TIME_WARP:       [${TIME_WARP}]"
    echo "COMMUNITY:       [${COMMUNITY}]"
    echo "VNAME:           [${VNAME}]"
    echo "GUI:             [${GUI}]"
    echo "ONBOARD:         [${ONBOARD}]"
    echo "AUTO_LAUNCHED:   [${AUTO_LAUNCHED}]"
    echo "BOLLARD:         [${BOLLARD}]"
    echo "------------------------------------------"
    echo "IP_ADDR:         [${IP_ADDR}]"
    echo "MOOS_PORT:       [${MOOS_PORT}]"
    echo "PSHARE_PORT:     [${PSHARE_PORT}]"
    echo "SHORE_IP:        [${SHORE_IP}]"
    echo "SHORE_PSHARE:    [${SHORE_PSHARE}]"
    echo "=========================================="
    echo -n "Hit any key to continue launch "
    read ANSWER
fi


#----------------------------------------------------------
# Part 5: Create the .moos and .bhv files
#------------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
NSFLAGS="--interactive --force"
fi

nsplug formula2boat.moos targ_boat.moos $NSFLAGS WARP=$TIME_WARP \
    IP_ADDR=$IP_ADDR MOOS_PORT=$MOOS_PORT \
    PSHARE_PORT=$PSHARE_PORT SHORE_IP=$SHORE_IP \
    SHORE_PSHARE=$SHORE_PSHARE ONBOARD=$ONBOARD BOLLARD=$BOLLARD\

if [ "${JUST_MAKE}" = "yes" ]; then
echo "$ME: Targ files made; exiting without launch."
exit 0
fi

#----------------------------------------------------------
#  Part 6: Launch the processes
#----------------------------------------------------------


#mission_dir=$PWD

# if [ ${ONBOARD} = "yes" ]; then

#     #Launch the remote pipereader
#     cd ~/Leucothea/MainCode/
#     xterm -e python3 remote_pipereader.py >& /dev/null &
    
#     # Launch the scripts that reads the NMEA data from seapath and send to virtual port
#     cd ${mission_dir}/../../scripts/
#     xterm -e python3 NMEA_receiver.py >& /dev/null &

#     cd ${mission_dir}
# fi


echo "Launching $COMMUNITY MOOS Community with WARP:" $TIME_WARP
pAntler targ_boat.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
echo "Done Launching $COMMUNITY MOOS Community"

#----------------------------------------------------------
# Part 7: If launched from script (launch.sh), we're done, exit now
#----------------------------------------------------------

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#----------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#----------------------------------------------------------

uMAC -t targ_boat.moos
trap "" SIGINT 
kill -- -$$