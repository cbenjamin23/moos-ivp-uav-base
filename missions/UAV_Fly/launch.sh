#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch.sh    
#   Author: Michael Benjamin   
#   LastEd: April 2022
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
RANDSTART="true"
VLAUNCH_ARGS="--auto --sim "
SLAUNCH_ARGS="--auto "

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                          "
	echo "                                                    "
	echo "Options:                                            "
        echo "  --help, -h                                        "
        echo "    Display this help message                       "
        echo "  --verbose, -v                                     "
        echo "    Increase verbosity                              "
	echo "  --just_make, -j                                   " 
	echo "    Just make the targ files, but do not launch.    " 
	echo "  --norand                                          " 
	echo "    Do not randomly generate vpositions.txt.        "
	echo "  --swim_file=<mit_00.txt>                          " 
	echo "    Set the swim file.                              "
	echo "                                                    "
	echo "  -1 :  Short for --swim_file=mit_01.txt            "
	echo "  -2 :  Short for --swim_file=mit_02.txt            "
	echo "  -3 :  Short for --swim_file=mit_03.txt            "
	echo "  -4 :  Short for --swim_file=mit_04.txt            "
	echo "  -5 :  Short for --swim_file=mit_05.txt            "
	echo "  -6 :  Short for --swim_file=mit_06.txt            "
	exit 0;
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--norand" -o "${ARGI}" = "-r" ]; then
	RANDSTART="false"

    elif [ "${ARGI:0:12}" = "--swim_file=" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-1" -o "${ARGI}" = "-2" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-3" -o "${ARGI}" = "-4" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-5" -o "${ARGI}" = "-6" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#-------------------------------------------------------------
# Part 4: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
vecho "Picking starting position"
if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    pickpos --poly="-2,-8 : 4,-13 : 60,13 : 57,18"   \
	    --amt=1  > vpositions.txt  
fi

# vehicle names are always deterministic in alphabetical order
pickpos --amt=1 --vnames  > vnames.txt

VEHPOS=(`cat vpositions.txt`)
VNAMES=(`cat vnames.txt`)

#-------------------------------------------------------------
# Part 5: Launch the vehicles
#-------------------------------------------------------------
INDEX=1

ARRAY_INDEX=`expr $INDEX - 1`

START=${VEHPOS[$ARRAY_INDEX]}
VNAME=${VNAMES[$ARRAY_INDEX]}

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`

IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
IX_VLAUNCH_ARGS+=" --start=$START "
IX_VLAUNCH_ARGS+=" --vname=$VNAME                "
IX_VLAUNCH_ARGS+=" --mport=$MOOS_PORT --pshare=$PSHARE_PORT "
IX_VLAUNCH_ARGS+=" $TIME_WARP $VERBOSE $JUST_MAKE"

vecho "Launching: $VNAME"
vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"

./launch_vehicle.sh $IX_VLAUNCH_ARGS

#-------------------------------------------------------------
# Part 6: Launch the Shoreside mission file
#-------------------------------------------------------------
SLAUNCH_ARGS+=" $JUST_MAKE"
vecho "Launching the shoreside. Args: $SLAUNCH_ARGS $TIME_WARP"

./launch_shoreside.sh $SLAUNCH_ARGS $VERBOSE $TIME_WARP 

if [ ${JUST_MAKE} != "" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------------
# Part 7: Launch uMac until the mission is quit
#-------------------------------------------------------------
uMAC --paused targ_shoreside.moos

kill -- -$$

