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


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/lib_common")

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Find the MOOSGeodesy library
find_package(MOOSGeodesy REQUIRED)


#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(lib_behaviors-test)
# ADD_SUBDIRECTORY(pExampleApp)
# ADD_SUBDIRECTORY(pXRelayTest)
# ADD_SUBDIRECTORY(pOdometry)
# ADD_SUBDIRECTORY(pPointAssign)
# ADD_SUBDIRECTORY(pGenPath)
# ADD_SUBDIRECTORY(pGenRescue)
ADD_SUBDIRECTORY(app_voiview)
ADD_SUBDIRECTORY(pArduBridge)
ADD_SUBDIRECTORY(lib_behaviors)
ADD_SUBDIRECTORY(pGridSearchViz)
ADD_SUBDIRECTORY(lib_fires)
ADD_SUBDIRECTORY(lib_ignoredRegions)
ADD_SUBDIRECTORY(uFldFireSim)


##############################################################################
#                           END of CMakeLists.txt
##############################################################################

