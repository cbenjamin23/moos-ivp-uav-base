#--------------------------------------------------------
# The CMakeLists.txt for:                lib_bhv_constAlt
# Author(s):                    Steve Carter Feujo Nomeny
#--------------------------------------------------------
#                                    BHV_ConstantAltitude
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



ADD_LIBRARY(BHV_ConstantAltitude SHARED 
   BHV_ConstantAltitude.cpp)

TARGET_LINK_LIBRARIES(BHV_ConstantAltitude
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${SYSTEM_LIBS}
)
