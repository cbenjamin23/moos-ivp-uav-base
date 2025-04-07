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



include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
# Build Library
ADD_LIBRARY(tmstc_star ${SRC})

target_link_libraries(tmstc_star
  geometry
)


# build test
ADD_EXECUTABLE(tmstc_example example_main.cpp)

# Specify include directories for the test executable
# TARGET_INCLUDE_DIRECTORIES(tmstc_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link the required libraries for the test executable
TARGET_LINK_LIBRARIES(tmstc_example
  tmstc_star
  geometry
  mbutil
  ${SYSTEM_LIBS}
)




