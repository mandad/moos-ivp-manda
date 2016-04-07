#--------------------------------------------------------
# The CMakeLists.txt for:                       iSonar
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(CMAKE_CXX_STANDARD 11)

SET(SRC
  Sonar.cpp
  Sonar_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iSonar ${SRC})

TARGET_LINK_LIBRARIES(iSonar
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   apputil
   mbutil
   m
   pthread)

