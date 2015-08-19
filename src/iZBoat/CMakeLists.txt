#--------------------------------------------------------
# The CMakeLists.txt for:                       iZBoat
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  ZBoat.cpp
  ZBoat_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iZBoat ${SRC})

TARGET_LINK_LIBRARIES(iZBoat
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

