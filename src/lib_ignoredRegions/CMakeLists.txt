#--------------------------------------------------------
# The CMakeLists.txt for:                lib_ignoredRegions
# Author(s):                                Steve Nomeny
#--------------------------------------------------------

SET(SRC
  IgnoredRegion.cpp
  IgnoredRegionSet.cpp
  IgnoredRegionGenerator.cpp
)

SET(HEADERS
  IgnoredRegion.h
  IgnoredRegionSet.h
  IgnoredRegionGenerator.h
)

# Build Library
ADD_LIBRARY(ignoredregions ${SRC})

