#--------------------------------------------------------
# The CMakeLists.txt for:                       pArduBridge
# Author(s):                              Steve Carter Feujo Nomeny
#--------------------------------------------------------

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

