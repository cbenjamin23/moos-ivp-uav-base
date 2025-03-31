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


message("ROBOT_APPS:" ${ROBOT_APPS})

#===============================================================
# Set the GUI Apps
#===============================================================
SET(GUI_APPS )

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/app_voiview)
  LIST(APPEND GUI_APPS app_voiview)
ENDIF()

#===============================================================
# Set the Shoreside Apps
#===============================================================
SET(SHORE_APPS)

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_tmstc_star)
  LIST(APPEND SHORE_APPS lib_tmstc_star)
ENDIF()

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_fires)
  LIST(APPEND SHORE_APPS lib_fires)
ENDIF()

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_ignoredRegions)
  LIST(APPEND SHORE_APPS lib_ignoredRegions)
ENDIF()

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pGridSearchViz)
  LIST(APPEND SHORE_APPS pGridSearchViz)
ENDIF()

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pGridSearchPlanner)
  LIST(APPEND SHORE_APPS pGridSearchPlanner)
ENDIF()

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/uFldFireSim)
  LIST(APPEND SHORE_APPS uFldFireSim)
ENDIF()

SET(APPS_TO_BUILD ${ROBOT_APPS})



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


