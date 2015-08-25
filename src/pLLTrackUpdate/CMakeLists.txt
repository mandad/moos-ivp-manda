#--------------------------------------------------------
# The CMakeLists.txt for:                 pLLTrackUpdate
# Author(s):                              Damian Manda
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  LLTrackUpdate.cpp
  LLTrackUpdate_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pLLTrackUpdate ${SRC})

TARGET_LINK_LIBRARIES(pLLTrackUpdate
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   geometry
   mbutil
   m
   pthread)

