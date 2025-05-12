#--------------------------------------------------------
# The CMakeLists.txt for:                     pGridSearchViz
# Author(s):                                Steve Nomeny 
#--------------------------------------------------------

SET(SRC
  MissionOperator.cpp
  MissionOperator_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pMissionOperator ${SRC})

TARGET_LINK_LIBRARIES(pMissionOperator
  ${MOOS_LIBRARIES}
   contacts
   geometry
   mbutil
   apputil
   ${SYSTEM_LIBS}
)

