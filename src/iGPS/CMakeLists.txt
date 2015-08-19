#--------------------------------------------------------
# The CMakeLists.txt for:                           iGPS
# Author(s):    Paul Newman, modified Andrew Patrikalakis
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(iGPS ${SRC})
   
TARGET_LINK_LIBRARIES(iGPS
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   m
   pthread)
