#--------------------------------------------------------
# The CMakeLists.txt for:                 pSurveyPath
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11)

SET(SRC
  SurveyPath.cpp
  SurveyPath_Info.cpp
  RecordSwath.cpp
  main.cpp
)

ADD_EXECUTABLE(pSurveyPath ${SRC})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} geos-config")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} geos-config")
#CFLAGS += `geos-config --cflags`
#LDFLAGS += `geos-config --ldflags`

TARGET_LINK_LIBRARIES(pSurveyPath
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread
   geos
)
