#--------------------------------------------------------
# The CMakeLists.txt for:                lib_common
# Author(s):                    Steve Carter Feujo Nomeny
#--------------------------------------------------------
#                                    common
#--------------------------------------------------------



ADD_LIBRARY(lib_common SHARED 
   common.cpp
)



TARGET_LINK_LIBRARIES(lib_common
   mbutil 
   geometry 
   ${SYSTEM_LIBS}
)
