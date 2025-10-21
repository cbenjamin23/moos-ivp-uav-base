##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2010/09/07
#              2020/05/09 minor mods
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})


# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Find the MOOSGeodesy library
find_package(MOOSGeodesy REQUIRED)


#========================================================================
# Let users control whether or not GUI-related libraries/apps are built.
#========================================================================
SET(BUILD_BOT_CODE_ONLY OFF CACHE BOOL "Build minimal set of bot apps.")


#===============================================================
# Set the ROBOT Apps
#===============================================================

SET(ROBOT_APPS)

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_behaviors )
  LIST(APPEND ROBOT_APPS lib_behaviors)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pArduBridge )
  LIST(APPEND ROBOT_APPS pArduBridge)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pProxonoi_uav )
  LIST(APPEND ROBOT_APPS pProxonoi_uav)
ENDIF()


message("ROBOT_APPS:" ${ROBOT_APPS})

SET(APPS_TO_BUILD ${ROBOT_APPS})


#===============================================================
# Set the GUI Apps
#===============================================================
SET(GUI_APPS )


#===============================================================
# Set the Shoreside Apps
#===============================================================
SET(SHORE_APPS)

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pMissionOperator)
  LIST(APPEND SHORE_APPS pMissionOperator)
ENDIF()


#===============================================================
# Build Apps
#===============================================================

message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
message(" BUILD_BOT_CODE_ONLY: ${BUILD_BOT_CODE_ONLY}")
message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

IF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")
  LIST(APPEND APPS_TO_BUILD ${GUI_APPS})
  LIST(APPEND APPS_TO_BUILD ${SHORE_APPS})
ENDIF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")

FOREACH(A ${APPS_TO_BUILD})
  message("APP:" ${A})
  ADD_SUBDIRECTORY(${A})
ENDFOREACH(A)


##############################################################################
#                           END of CMakeLists.txt
##############################################################################


