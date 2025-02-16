#--------------------------------------------------------
# The CMakeLists.txt for:                lib_common
# Author(s):                    Steve Carter Feujo Nomeny
#--------------------------------------------------------
#                                    common
#--------------------------------------------------------



ADD_LIBRARY(lib_common SHARED 
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
