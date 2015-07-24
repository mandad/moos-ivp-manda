#--------------------------------------------------------
# The CMakeLists.txt for:                       pSurveyPath
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  SurveyPath.cpp
  SurveyPath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSurveyPath ${SRC})

TARGET_LINK_LIBRARIES(pSurveyPath
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

