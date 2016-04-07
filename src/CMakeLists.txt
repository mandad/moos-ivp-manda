##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2015/08/26
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(lib_anrp_util)

ADD_SUBDIRECTORY(iMOOSArduino)
ADD_SUBDIRECTORY(pMOOSArduinoTester)
ADD_SUBDIRECTORY(uMOOSArduinoLib)
ADD_SUBDIRECTORY(iGP9)
ADD_SUBDIRECTORY(iZBoat)
ADD_SUBDIRECTORY(iGPS)
ADD_SUBDIRECTORY(iGPS_MB1)
ADD_SUBDIRECTORY(pLLTrackUpdate)
ADD_SUBDIRECTORY(iCV100)
ADD_SUBDIRECTORY(pSonarFilter)
ADD_SUBDIRECTORY(pMarineMRAS)
ADD_SUBDIRECTORY(pSurveyPath)
ADD_SUBDIRECTORY(uSimNomoto)
ADD_SUBDIRECTORY(iSonar)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
