#--------------------------------------------------------
# The CMakeLists.txt for:                       pMarineMRAS
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  MarineMRAS.cpp
  MarineMRAS_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pMarineMRAS ${SRC})

TARGET_LINK_LIBRARIES(pMarineMRAS
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

