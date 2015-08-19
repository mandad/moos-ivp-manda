#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_anrp_util
# Author(s):                          Andrew Patrikalakis
#--------------------------------------------------------

FILE(GLOB SRC *.cpp *.c)

# Replaced SHARED with a variable because cross-compiling for my TS-7800 ARM board requires static linking.
ADD_LIBRARY(anrp_util ${BUILD_STATIC_OR_SHARED_LIBRARIES} ${SRC})

TARGET_LINK_LIBRARIES(anrp_util
${MOOS_LIBRARIES})
