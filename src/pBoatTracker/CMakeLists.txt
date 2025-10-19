#--------------------------------------------------------
# The CMakeLists.txt for:                       pBoatTracker
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  BoatTracker.cpp
  BoatTracker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBoatTracker ${SRC})

TARGET_LINK_LIBRARIES(pBoatTracker
   ${MOOS_LIBRARIES}
   contacts
    geometry
    apputil
    mbutil
    m
    pthread
   )

