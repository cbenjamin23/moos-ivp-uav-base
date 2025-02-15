#--------------------------------------------------------
# The CMakeLists.txt for:                     app_voiview
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS m)
endif (${WIN32})

# Set GL Libraries
if (${APPLE})
  SET(GL_LIBS fltk_gl)
else (${APPLE})
  SET(GL_LIBS fltk_gl GL)
endif (${APPLE})



SET(SRC 
  VOI_Viewer.cpp
  VOI_GUI.cpp
  main.cpp
  )

ADD_EXECUTABLE(voiview ${SRC})


target_include_directories(voiview
  PRIVATE
    ${MOOSIVP_SOURCE_TREE_BASE}/include/ivp
    ${MOOSIVP_SOURCE_TREE_BASE}/MOOS/MOOSGeodesy/libMOOSGeodesy/include/
)


TARGET_LINK_LIBRARIES(voiview 
  marineview
  MOOSGeodesy
  voronoi
  genpoly
  genutil
  geometry
  contacts
  mbutil
  fltk 
  ${GL_LIBS}
  dl
  tiff
  ${SYSTEM_LIBS}
)
  
# http://developer.apple.com/qa/qa2007/qa1567.html
IF (${APPLE})
  SET_TARGET_PROPERTIES(voiview PROPERTIES	  
    LINK_FLAGS "-framework OpenGL -framework AGL -framework Cocoa"
    )
ENDIF (${APPLE})

