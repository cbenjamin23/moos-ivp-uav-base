#--------------------------------------------------------
# The CMakeLists.txt for:     lib_bhv_task_refuel_replace_basic
# Author(s):                             Charles Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

#--------------------------------------------------------
#                                   BHV_TaskRefuelReplaceBasic
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskRefuelReplaceBasic SHARED
   BHV_TaskRefuelReplaceBasic.cpp )

TARGET_INCLUDE_DIRECTORIES(BHV_TaskRefuelReplaceBasic PUBLIC
  ${MOOSIVP_SWARM_TREE_BASE}/src/lib_helmtask
)
TARGET_LINK_LIBRARIES(BHV_TaskRefuelReplaceBasic
   helmtask
   ufield
   helmivp
   contacts
   behaviors
   ivpbuild
   logic
   ivpcore
   bhvutil
   mbutil
   geometry
   ${SYSTEM_LIBS} )
