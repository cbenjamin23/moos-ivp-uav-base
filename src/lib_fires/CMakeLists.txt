#--------------------------------------------------------
# The CMakeLists.txt for:                    lib_fires
# Author(s):                                Steve Nomeny
#--------------------------------------------------------

SET(SRC
  FireFldGenerator.cpp
  Fire.cpp
  FireSet.cpp
  FireMissionScorer.cpp
)

SET(HEADERS
  FireFldGenerator.h
  Fire.h
  FireSet.h
  FireMissionScorer.h
)

# Build Library
ADD_LIBRARY(fires ${SRC})

