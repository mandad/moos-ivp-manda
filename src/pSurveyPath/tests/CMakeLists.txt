#--------------------------------------------------------
# The CMakeLists.txt for:                 SurveyPath tests
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11)

set(TEST_SRCS
  ../PathPlan.cpp
  ../RecordSwath.cpp
  TestPathPlan.cpp
  TestMain.cpp
  )

set(TEST_EXE "test-pathplan")

ADD_EXECUTABLE(${TEST_EXE} ${TEST_SRCS})

TARGET_LINK_LIBRARIES(${TEST_EXE}
  ${MOOS_LIBRARIES}
  apputil
  geometry
  mbutil
  m
  geos
   )


 # Add a target to actually run the tests.
#add_custom_target(catch COMMAND ${TEST_EXE})

add_test(NAME test-pathplan COMMAND ${TEST_EXE})
