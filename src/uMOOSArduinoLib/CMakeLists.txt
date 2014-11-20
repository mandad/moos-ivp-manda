#--------------------------------------------------------
# The CMakeLists.txt for:                       pMOOSArduinoTester
# Author(s):                              Mike Bogochow
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  FIFOComm.cpp
  IMOOSComm.cpp
  ArduinoComm.cpp
  CommType.cpp
  NetComm.cpp
  NetClientComm.cpp
  NetServerComm.cpp
)

add_library(comm_lib FIFOComm.cpp IMOOSComm.cpp ArduinoComm.cpp CommType.cpp NetClientComm.cpp NetServerComm.cpp)

#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(NetUtil)

#ADD_EXECUTABLE(pMOOSArduinoTester ${SRC})

#TARGET_LINK_LIBRARIES(uMOOSArduinoLib
#   mbutil
#   m
#   pthread)
