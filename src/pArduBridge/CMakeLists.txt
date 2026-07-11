#--------------------------------------------------------
# The CMakeLists.txt for:                       pArduBridge
# Author(s):                              Steve Carter Feujo Nomeny
#--------------------------------------------------------

# Set the build type to Debug to include debugging symbols
set(CMAKE_BUILD_TYPE Debug)

# Add the -g flag explicitly for debugging
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Suppress warnings from external MAVSDK/mavlink headers
add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)

SET(SRC
  ArduBridge.cpp
  ArduBridge_Info.cpp
  UAV_Model.cpp
  main.cpp
)

ADD_EXECUTABLE(pArduBridge ${SRC})


TARGET_LINK_LIBRARIES(pArduBridge
  MAVSDK::mavsdk  # Link against MAVSDK
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  contacts
  geometry 
  apputil
  mbutil
  m
  pthread)

IF(BUILD_TESTING)
  ADD_EXECUTABLE(pArduBridge_mode_confirmation_test
    tests/ModeConfirmationTrackerTest.cpp)
  TARGET_INCLUDE_DIRECTORIES(pArduBridge_mode_confirmation_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  ADD_TEST(NAME pArduBridge_mode_confirmation
    COMMAND pArduBridge_mode_confirmation_test)

  ADD_EXECUTABLE(pArduBridge_uav_policy_test
    tests/UAVPolicyTest.cpp
    UAV_Model.cpp)
  TARGET_INCLUDE_DIRECTORIES(pArduBridge_uav_policy_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  TARGET_LINK_LIBRARIES(pArduBridge_uav_policy_test
    MAVSDK::mavsdk
    ${MOOS_LIBRARIES}
    ${MOOSGeodesy_LIBRARIES}
    contacts
    geometry
    apputil
    mbutil
    m
    pthread)
  ADD_TEST(NAME pArduBridge_uav_policy
    COMMAND pArduBridge_uav_policy_test)
ENDIF()
