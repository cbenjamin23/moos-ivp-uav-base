#--------------------------------------------------------
# The CMakeLists.txt for:     lib_bhv_task_refuel_replace_target
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
#                                   BHV_TaskRefuelReplaceTarget
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskRefuelReplaceTarget SHARED 
   BHV_TaskRefuelReplaceTarget.cpp )

TARGET_INCLUDE_DIRECTORIES(BHV_TaskRefuelReplaceTarget PUBLIC
  ${MOOSIVP_SWARM_TREE_BASE}/src/lib_helmtask
)
TARGET_LINK_LIBRARIES(BHV_TaskRefuelReplaceTarget
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
