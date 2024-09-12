#--------------------------------------------------------
# The CMakeLists.txt for:                      pGenRescue
# Author(s):             Steve Nomeny and Filip Stromstad
#--------------------------------------------------------

SET(SRC
  GenRescue.cpp
  GenRescue_Info.cpp
  main.cpp
)

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)


ADD_EXECUTABLE(pGenRescue ${SRC})

TARGET_LINK_LIBRARIES(pGenRescue
   MAVSDK::mavsdk  # Link against MAVSDK
   ${MOOS_LIBRARIES}
   contacts
   apputil
   geometry
   mbutil
   m
   pthread
   ufield
   )

