 #!/bin/bash
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
SCENARIO=1
JUST_MAKE="no"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_build, -j    \n"
	printf "  --help, -h         \n"
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then
        SCENARIO=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
	JUST_MAKE="yes"
    else
	printf "Bad Argument: %s \n" $ARGI
	exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files.
#-------------------------------------------------------
# 1 = Testing Controls on Swains
if [ $SCENARIO = 1 ]; then
  # MOOS Stuff
  START_POS="x=0,y=0,speed=0,heading=90"
  LAT_ORIGIN=43.2001957
  LONG_ORIGIN=-71.0466533
  TIFF_FILE=Swains_Aerial.tif
  TIFF_FILE_B=Swains_Aerial.tif
  PAN_X=300
  PAN_Y=-1300
  ZOOM=0.65
  # Shore Config
  #SHORE_IP="10.42.0.115"
  SHORE_IP="192.168.1.26"
  SHORE_PORT=9300
  #In this case, the RPi
  #BOAT_MODEM_IP="10.42.0.11"
  BOAT_MODEM_IP="192.168.1.243"
  BOAT_PORT=9301
  ASV_NAME="ASV3"
  #Behavior
  HOME_POS="0,-60"
  HOME_INNER_RADIUS=2
  HOME_OUTER_RADIUS=5
  SURVEY_SPEED=0.2
  TEST_SPEED=0.2
  BOAT_LENGTH=3
  SIMULATION=FALSE
  #BHV_FILE=targ_consthead_pattern.bhv
fi
# What is nsplug? Type "nsplug --help" or "nsplug --manual"

#===================================================
# New Style meta files
#===================================================

# Mission Configs
#---------------------------------------------------

nsplug meta_test_mras.moos targ_test_mras.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BOAT_LENGTH=$BOAT_LENGTH ASV_NAME=$ASV_NAME MANUAL_CONTROL=FALSE \
   BHV_FILE=targ_consthead_pattern.bhv

nsplug meta_test_mras.moos targ_manual_control.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   BOAT_LENGTH=$BOAT_LENGTH  ASV_NAME=$ASV_NAME \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION \
   MANUAL_CONTROL=TRUE

nsplug meta_test_mras.moos targ_test_circle.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BOAT_LENGTH=$BOAT_LENGTH ASV_NAME=$ASV_NAME MANUAL_CONTROL=FALSE \
   BHV_FILE=targ_circle_pattern.bhv

nsplug meta_test_mras.moos targ_test_lines.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BOAT_LENGTH=$BOAT_LENGTH ASV_NAME=$ASV_NAME MANUAL_CONTROL=FALSE \
   BHV_FILE=targ_lines.bhv

nsplug meta_laptop.moos targ_laptop.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN   LONG_ORIGIN=$LONG_ORIGIN \
   TIFF_FILE="$TIFF_FILE"  PAN_X=$PAN_X  PAN_Y=$PAN_Y  ZOOM=$ZOOM \
   SHORE_IP=$SHORE_IP  SHORE_PORT=$SHORE_PORT  TIFF_FILE_B=$TIFF_FILE_B \
   START_POS=$START_POS  ASV_NAME=$ASV_NAME

# Behaviors
#---------------------------------------------------

nsplug meta_consthead_pattern.bhv targ_consthead_pattern.bhv --path=../shared_plugins/behavior \
  -f HOME_POS=$HOME_POS \
  SURVEY_SPEED=$SURVEY_SPEED TEST_SPEED=$TEST_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

nsplug meta_lines.bhv targ_lines.bhv --path=../shared_plugins/behavior \
 -f HOME_POS=$HOME_POS \
  SURVEY_SPEED=$SURVEY_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

nsplug meta_circle_pattern.bhv targ_circle_pattern.bhv --path=../shared_plugins/behavior \
 -f HOME_POS=$HOME_POS TEST_SPEED=$TEST_SPEED \
  SURVEY_SPEED=$SURVEY_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

