#--------------------------------------------------------
# The CMakeLists.txt for:                       uRecordSwath
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  RecordSwath.cpp
  RecordSwath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uRecordSwath ${SRC})

TARGET_LINK_LIBRARIES(uRecordSwath
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

