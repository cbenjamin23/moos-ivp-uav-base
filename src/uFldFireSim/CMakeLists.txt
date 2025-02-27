#--------------------------------------------------------
# The CMakeLists.txt for:                   uFldFireSim
# Author(s):                                Steve Nomeny
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    m
    pthread)
endif (${WIN32})

SET(SRC
   FireSim.cpp
   FireSim_Info.cpp
   main.cpp
)

ADD_EXECUTABLE(uFldFireSim ${SRC})

TARGET_LINK_LIBRARIES(uFldFireSim
  fires
  ${MOOS_LIBRARIES}
  contacts
  geometry 
  mbutil
  apputil
  ${SYSTEM_LIBS}
)


