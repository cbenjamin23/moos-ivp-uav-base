#--------------------------------------------------------
# The CMakeLists.txt for:                       pArduBridge
# Author(s):                              Steve Carter Feujo Nomeny
#--------------------------------------------------------

SET(SRC
  ArduBridge.cpp
  ArduBridge_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pArduBridge ${SRC})

TARGET_LINK_LIBRARIES(pArduBridge
  MAVSDK::mavsdk  # Link against MAVSDK
  ${MOOS_LIBRARIES}
  apputil
  mbutil
  m
  pthread)

