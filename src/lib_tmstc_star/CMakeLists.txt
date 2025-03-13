#--------------------------------------------------------
# The CMakeLists.txt for:                lib_tmstc_star
# Author(s):                             Steve C. F. Nomeny
#--------------------------------------------------------
# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    m
    pthread)
endif (${WIN32})

file(GLOB SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

# SET(SRC
#   ${CMAKE_CURRENT_SOURCE_DIR}/src
#   # ${CMAKE_CURRENT_SOURCE_DIR}/src/ACO_STC.cpp 
#   # ${CMAKE_CURRENT_SOURCE_DIR}/src/Dinic.cpp 
#   # ${CMAKE_CURRENT_SOURCE_DIR}/src/HeuristicPartition.cpp 
#   # ${CMAKE_CURRENT_SOURCE_DIR}/src/MaximumSubRectDivision.cpp 
#   # ${CMAKE_CURRENT_SOURCE_DIR}/src/PathCut.cpp 
# )
# not used
# /src/m_TSP.cpp 
# /src/MapToMatrix.cpp

# SET(HEADERS
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# # only these headers are used
# # ${CMAKE_CURRENT_SOURCE_DIR}/include/ACO_STC.h 
# # ${CMAKE_CURRENT_SOURCE_DIR}/include/Dinic.h 
# # ${CMAKE_CURRENT_SOURCE_DIR}/include/HeuristicPartition.h 
# # ${CMAKE_CURRENT_SOURCE_DIR}/include/MaximumSubRectDivision.h 
# # ${CMAKE_CURRENT_SOURCE_DIR}/include/PathCut.h 
# )

# no need for these headers
# /include/ConnectComponent.h 
# /include/DARP.h 
# /include/DARPPlanner.h 
# /include/m_TSP.h 
# /include/STC.h 





include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
# Build Library
ADD_LIBRARY(tmstc_star ${SRC})



# build test
ADD_EXECUTABLE(tmstc_example example_main.cpp)

# Specify include directories for the test executable
# TARGET_INCLUDE_DIRECTORIES(tmstc_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link the required libraries for the test executable
TARGET_LINK_LIBRARIES(tmstc_example
  tmstc_star
  ${SYSTEM_LIBS}
)




