#--------------------------------------------------------
# The CMakeLists.txt for:                       pProxonoi
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  Proxonoi.cpp
  Proxonoi_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pProxonoi_uav ${SRC})

target_include_directories(pProxonoi_uav
  PRIVATE
    ${MOOSIVP_SOURCE_TREE_BASE}/include/ivp
    ${MOOSIVP_SOURCE_TREE_BASE}/MOOS/MOOSGeodesy/libMOOSGeodesy/include/
)


TARGET_LINK_LIBRARIES(pProxonoi_uav
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   voronoi
   ufield
   apputil
   contacts
   geometry
   mbutil
   m
   pthread)

