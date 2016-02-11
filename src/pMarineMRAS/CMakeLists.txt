#--------------------------------------------------------
# The CMakeLists.txt for:                       pMarineMRAS
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11) 

SET(SRC
  MarineMRAS.cpp
  MarineMRAS_Info.cpp
  CourseChangeMRAS.cpp
  CourseKeepMRAS.cpp
  SpeedControl.cpp
  ThrustMap.cpp
  CurrentEstimate.cpp
  main.cpp
)

ADD_EXECUTABLE(pMarineMRAS ${SRC})

TARGET_LINK_LIBRARIES(pMarineMRAS
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)

