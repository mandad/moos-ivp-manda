#--------------------------------------------------------
# The CMakeLists.txt for:                       iMOOSArduino
# Author(s):                              Mike Bogochow, Jeff Masucci, Cody Noel
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  MOOSArduino.cpp
  MOOSArduino_Info.cpp
  UtilMOOSArduino.cpp
  main.cpp
)

ADD_EXECUTABLE(iMOOSArduino ${SRC})

TARGET_LINK_LIBRARIES(iMOOSArduino
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread
   comm_lib
   )

