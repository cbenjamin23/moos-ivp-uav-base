#!/bin/bash 
#-------------------------------------------------------
#  Script: launch_all.sh                       
#  Author: Charles Benjamin
#  Mission: formula2boat
#  LastEd: 06/2025
#  Description: Launches the MOOS community for the vehicle and shoreside communities
#  Usage: launch_all.sh [WARP]
#  Note: This script is only used for making simulation more efficient.
#-------------------------------------------------------
#  Part 1: Set convenience functions and exit actions
#----------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi; }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; rm -f targ_*.moos; }
trap on_exit SIGINT 

# --------------------------------------------------------
# Part 2: Set global variables and defaults
#--------------------------------------------------------

TIME_WARP=1
JUST_MAKE=""
XLAUNCHED="no"
NOGUI=""

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
      echo "launch_all.sh [SWITCHES] [WARP]               " 
      echo "  --help, -h                                  " 
      echo "    Display this help message                 "
      echo "  --just_make, -j                             " 
      echo "    Just make targ files, but do not launch   "
      echo "                                              "
      exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="-j"
    else 
      echo "launch_all.sh: Bad Arg:" $ARGI "Exit Code 1."
      exit 1
    fi
done

#-------------------------------------------------------
#  Part 4: Launch the vehicle
#-------------------------------------------------------

echo "Launching RAN...."
./launch_vehicle.sh --nogui $JUST_MAKE $TIME_WARP --auto &

#-------------------------------------------------------
#  Part 5: Launch the shoreside
#-------------------------------------------------------
echo "Launching shoreside...."
./launch_shoreside.sh $JUST_MAKE $TIME_WARP --auto &

if [ "${XLAUNCHED}" != "yes" ]; then
    uMAC --paused shoreside.moos
    trap "" SIGINT
    echo; echo "$ME: Halting all apps"
    kill -- -$$
fi

exit 0