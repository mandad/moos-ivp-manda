#--------------------------------------------------------
# The CMakeLists.txt for:                      iCV100
# Author(s):  Damian Manda 
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(iCV100 ${SRC})
   
TARGET_LINK_LIBRARIES(iCV100
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   anrp_util
   m
   pthread)
