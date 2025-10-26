#--------------------------------------------------------
# The CMakeLists.txt for:                       pArduBridge
# Author(s):                              Steve Carter Feujo Nomeny
#--------------------------------------------------------

# Set the build type to Debug to include debugging symbols
set(CMAKE_BUILD_TYPE Debug)

# Add the -g flag explicitly for debugging
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Suppress warnings from external MAVSDK/mavlink headers
add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)

SET(SRC
  ArduBridge.cpp
  ArduBridge_Info.cpp
  UAV_Model.cpp
  main.cpp
)

ADD_EXECUTABLE(pArduBridge ${SRC})


TARGET_LINK_LIBRARIES(pArduBridge
  MAVSDK::mavsdk  # Link against MAVSDK
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  contacts
  geometry 
  apputil
  mbutil
  m
  pthread)

