#--------------------------------------------------------
# The CMakeLists.txt for:                 SurveyPath tests
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11)

set(TEST_SRCS
  ../PathPlan.cpp
  ../RecordSwath.cpp
  TestPathPlan.cpp
  TestRecordSwath.cpp
  TestMain.cpp
  )

FIND_PACKAGE(Boost COMPONENTS system filesystem iostreams REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS}  )

set(TEST_EXE "test-surveypath")

ADD_EXECUTABLE(${TEST_EXE} ${TEST_SRCS})

TARGET_LINK_LIBRARIES(${TEST_EXE}
  ${MOOS_LIBRARIES}
  ${Boost_LIBRARIES}
  apputil
  geometry
  mbutil
  m
  geos
   )


 # Add a target to actually run the tests.
#add_custom_target(catch COMMAND ${TEST_EXE})

add_test(NAME test-surveypath COMMAND ${TEST_EXE})
