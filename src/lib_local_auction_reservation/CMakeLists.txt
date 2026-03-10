#--------------------------------------------------------
# The CMakeLists.txt for:     lib_local_auction_reservation
#--------------------------------------------------------

if (${WIN32})
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

ADD_LIBRARY(local_auction_reservation SHARED
  LocalAuctionReservation.cpp)

TARGET_LINK_LIBRARIES(local_auction_reservation
  ${SYSTEM_LIBS})
