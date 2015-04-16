#--------------------------------------------------------
# The CMakeLists.txt for:                       iGP9
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  GP9.cpp
  GP9_Info.cpp
  comms.cpp
  registers.cpp
  main.cpp
)

ADD_EXECUTABLE(iGP9 ${SRC})

TARGET_LINK_LIBRARIES(iGP9
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   serial)

