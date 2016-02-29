#--------------------------------------------------------
# The CMakeLists.txt for:                 pSurveyPath
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11)

# Find package is broken under < 3.3 on Homebrew
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  include_directories(/usr/local/Cellar/eigen/3.2.8/include/eigen3/)
endif()

SET(SRC
  SurveyPath.cpp
  SurveyPath_Info.cpp
  RecordSwath.cpp
  PathPlan.cpp
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

# Add the tests at the end so they can find the eigen library
ADD_SUBDIRECTORY(tests)
