#--------------------------------------------------------
# The CMakeLists.txt for:                       pSonarFilter
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  SonarFilter.cpp
  SonarFilter_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSonarFilter ${SRC})

TARGET_LINK_LIBRARIES(pSonarFilter
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

