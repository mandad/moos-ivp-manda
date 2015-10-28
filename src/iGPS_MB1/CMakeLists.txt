#--------------------------------------------------------
# The CMakeLists.txt for:                       iGPS_UDP
# Author(s):    Paul Newman, modified Andrew Patrikalakis, Damian Manda
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(iGPS_MB1 ${SRC})
   
TARGET_LINK_LIBRARIES(iGPS_MB1
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   m
   pthread)
