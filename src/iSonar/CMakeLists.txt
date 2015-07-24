#--------------------------------------------------------
# The CMakeLists.txt for:                       iSonar
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  Sonar.cpp
  Sonar_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iSonar ${SRC})

TARGET_LINK_LIBRARIES(iSonar
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

