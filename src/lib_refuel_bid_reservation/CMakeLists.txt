#--------------------------------------------------------
# The CMakeLists.txt for:     lib_refuel_bid_reservation
#--------------------------------------------------------

if (${WIN32})
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

ADD_LIBRARY(refuel_bid_reservation SHARED
  RefuelBidReservation.cpp)

TARGET_LINK_LIBRARIES(refuel_bid_reservation
  ${SYSTEM_LIBS})
