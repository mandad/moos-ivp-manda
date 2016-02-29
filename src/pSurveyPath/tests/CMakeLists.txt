#--------------------------------------------------------
# The CMakeLists.txt for:                 SurveyPath tests
# Author(s):                              Damian Manda
#--------------------------------------------------------
SET(CMAKE_CXX_STANDARD 11)

FILE(GLOB SRC *.cpp)

set(TEST_EXE "test-pathplan")

ADD_EXECUTABLE(${TEST_EXE} ${SRC})

TARGET_LINK_LIBRARIES(${TEST_EXE}
   #pSurveyPath
   )


 # Add a target to actually run the tests.
#add_custom_target(catch COMMAND ${TEST_EXE})

add_test(NAME test-pathplan COMMAND test-pathplan)

enable_testing(true)
