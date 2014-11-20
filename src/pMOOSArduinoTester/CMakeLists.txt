#--------------------------------------------------------
# The CMakeLists.txt for:                       pMOOSArduinoTester
# Author(s):                              Mike Bogochow
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  MOOSArduinoTester.cpp
  MOOSArduinoTester_Info.cpp
  ../iMOOSArduino/UtilMOOSArduino.cpp
  main.cpp
  ../uMOOSArduinoLib/FIFOComm.cpp
)



ADD_EXECUTABLE(pMOOSArduinoTester ${SRC})

TARGET_LINK_LIBRARIES(pMOOSArduinoTester
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)
