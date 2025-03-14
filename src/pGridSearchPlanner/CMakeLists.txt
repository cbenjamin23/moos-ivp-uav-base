#--------------------------------------------------------
# The CMakeLists.txt for:                     pGridSearchViz
# Author(s):                                Steve Nomeny 
#--------------------------------------------------------

SET(SRC
  GridSearchPlanner.cpp
  GridSearchPlanner_Info.cpp
  TMSTCGridConverter.cpp
  main.cpp
)

ADD_EXECUTABLE(pGridSearchPlanner ${SRC})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../lib_tmstc_star/include")

TARGET_LINK_LIBRARIES(pGridSearchPlanner
  tmstc_star
  ignoredregions
  ${MOOS_LIBRARIES}
   bhvutil
   contacts
   geometry
   mbutil
   apputil
   ${SYSTEM_LIBS}
)

