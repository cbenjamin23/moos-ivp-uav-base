#!/bin/bash

#--------------------------------------------------------------
# Set Global variables                               
#-------------------------------------------------------------- 
INVOCATION_ABS_DIR=`pwd`
BUILD_TYPE="None"
CMD_LINE_ARGS=""
BUILD_BOT_CODE_ONLY="OFF"
FORCE_FULL_RASPI_BUILD="NO"

#-------------------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "build.sh [SWITCHES]                         "
	echo "Switches:                                   " 
	echo "  --help, -h                                " 
        echo "  --debug,   -d                             "
	echo "  --force, -f                               "
	echo "    Force build full code even on RasPi     "
	echo "  --minrobot, -m                            "
	echo "    Only build minimal robot apps           "
	echo "  --minrobotx, -mx                          "
	echo "    Override min-robot default on Raspbian  "
	echo "  --release, -r                             "
	echo "Notes:                                      "
	echo " (1) All other command line args will be passed as args    "
	echo "     to \"make\" when it is eventually invoked.            "
	echo " (2) For example -k will continue making when/if a failure "
	echo "     is encountered in building one of the subdirectories. "
	echo " (3) For example -j2 will utilize a 2nd core in the build  "
	echo "     if your machine has two cores. -j4 etc for quad core. "
	exit 0;
    elif [ "${ARGI}" = "--debug" -o "${ARGI}" = "-d" ] ; then
        BUILD_TYPE="Debug"
    elif [ "${ARGI}" = "--release" -o "${ARGI}" = "-r" ] ; then
        BUILD_TYPE="Release"
    elif [ "${ARGI}" = "--minrobot" -o "${ARGI}" = "-m" ] ; then
        BUILD_BOT_CODE_ONLY="ON"
    elif [ "${ARGI}" = "--minrobotx" -o "${ARGI}" = "-mx" ]; then
        FORCE_FULL_RASPI_BUILD="yes"
    else
	CMD_LINE_ARGS=$CMD_LINE_ARGS" "$ARGI
    fi
done




#-------------------------------------------------------------- 
#  Part 2: If this is Odroid and minrobot not selected, and
#          no explicit override given with -mx, CONFIRM first
# 		   Check if the alias 'WhichOdroid' exists
#-------------------------------------------------------------- 
if [ -n "$WHICH_ODROID" ] && [ "${BUILD_BOT_CODE_ONLY}" = "OFF" ]; then
    if [ ! "${FORCE_FULL_ODROID_BUILD}" = "yes" ]; then
	echo "Odroid detected without --minrobotx or -mx selected."
	echo "[y] Continue with full build"
	echo "[M] Continue with minrobot build"
	echo -n "Continue? [y/M] "
	read ANSWER
	if [ ! "${ANSWER}" = "y" ]; then
	    BUILD_BOT_CODE_ONLY="ON"
	fi
    fi
fi


#---------------------------------------------------------
# Part 3: Set Compiler flags
#---------------------------------------------------------
# CMAKE_CXX_FLAGS="-Wall -Wextra "
CMAKE_CXX_FLAGS="-Wno-unused-parameter "
CMAKE_CXX_FLAGS+="-Wno-missing-field-initializers -pedantic -fPIC "
CMAKE_CXX_FLAGS+="-Wno-deprecated-declarations "
CMAKE_CXX_FLAGS+="-Wno-reorder "
CMAKE_CXX_FLAGS+="-Wno-cast-function-type "
CMAKE_CXX_FLAGS+="-Wno-cast-function-type "

#-------------------------------------------------------------- 
#  Part 4: Invoke the call to make in the build directory
#-------------------------------------------------------------- 

mkdir -p build
cd build

cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
	  -DCMAKE_BUILD_TYPE=${BUILD_TYPE}                    \
      -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS}"              \
      -DBUILD_BOT_CODE_ONLY=${BUILD_BOT_CODE_ONLY}  ../  || exit 1

make ${CMD_LINE_ARGS}
cd ${INVOCATION_ABS_DIR}


