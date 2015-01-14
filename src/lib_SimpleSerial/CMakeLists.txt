#--------------------------------------------------------
# The CMakeLists.txt for:                lib_SimpleSerial
# Author(s):                                   Alon Yaari
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

FILE(GLOB HEADERS *.h)

# Build Library
ADD_LIBRARY(SimpleSerial ${SRC})

# Install Headers
#INSTALL( FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/lib_SimpleSerial)

# Install Libararies
INSTALL( TARGETS SimpleSerial
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
   LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
   ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
