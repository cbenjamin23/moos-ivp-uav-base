#--------------------------------------------------------
# The CMakeLists.txt for:                  pRefuelReplace
# Author(s):                             Charles Benjamin
#--------------------------------------------------------

SET(SRC
  RefuelReplace.cpp
  RefuelReplace_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pRefuelReplace ${SRC})

TARGET_LINK_LIBRARIES(pRefuelReplace
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

