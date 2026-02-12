#--------------------------------------------------------
# The CMakeLists.txt for:                     pGridSearchViz
# Author(s):                                Steve Nomeny 
#--------------------------------------------------------

SET(SRC
  GridSearchViz.cpp
  GridSearchViz_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGridSearchViz ${SRC})

TARGET_LINK_LIBRARIES(pGridSearchViz
  ignoredregions
  ${MOOS_LIBRARIES}
   bhvutil
   contacts
   geometry
   mbutil
   apputil
   ${SYSTEM_LIBS}
)

