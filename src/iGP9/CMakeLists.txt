#--------------------------------------------------------
# The CMakeLists.txt for:                       iGP9
# Author(s):                              Damian Manda
#--------------------------------------------------------

SET(SRC
  GP9.cpp
  GP9_Info.cpp
  comms.cpp
  registers.cpp
  main.cpp
)

#FILE(GLOB SRC *.cpp)

ADD_DEFINITIONS(
  -std=c++11
)

ADD_EXECUTABLE(iGP9 ${SRC})

TARGET_LINK_LIBRARIES(iGP9
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread
   serial)

if(APPLE)
add_custom_command(TARGET iGP9
    POST_BUILD COMMAND 
    ${CMAKE_INSTALL_NAME_TOOL} -change libserial.dylib @executable_path/libserial.dylib $<TARGET_FILE:iGP9>
    )
endif()
