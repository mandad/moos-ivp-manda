 #!/bin/bash
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
SCENARIO=4
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
# 1 = Chincoteague, VA
# 2 = Test at home
# 3 = Field in The Hague, Norfolk
# 4 = Lake Bradford, VA
if [ $SCENARIO = 1 ]; then
  #MOOS Stuff
  START_POS="3000,8240"
  LAT_ORIGIN=37.7499688
  LONG_ORIGIN=-75.4585006
  TIFF_FILE=data/Chincoteague_Ref.tif
  TIFF_FILE_B=data/Chincoteague_Ref.tif
  PAN_X=-500
  PAN_Y=-1100
  ZOOM=1.35
  SHORE_IP="73.219.106.147"
  SHORE_PORT=9300
  # BOAT_MODEM_IP="166.150.166.136"
  BOAT_MODEM_IP="166.150.167.176"
  BOAT_PORT=9301
  #Behavior
  HOME_POS="3510,9080"
  HOME_INNER_RADIUS=5
  HOME_OUTER_RADIUS=20
  SURVEY_SPEED=1.6
  SIMULATION=FALSE
  BHV_FILE=targ_zboat_lines.bhv
fi

if [ $SCENARIO = 2 ]; then
  #MOOS Stuff
  START_POS="150,129"
  LAT_ORIGIN=36.8525995
  LONG_ORIGIN=-76.3003448
  TIFF_FILE=data/Norfolk_Chart.tif
  TIFF_FILE_B=data/Norfolk_Aerial.tif
  PAN_X=-322
  PAN_Y=-284
  ZOOM=1.4
  #In this case, the shore VM
  SHORE_IP="192.168.1.166"
  SHORE_PORT=9300
  #In this case, the RPi
  BOAT_MODEM_IP="192.168.1.243"
  BOAT_PORT=9301
  #Behavior
  HOME_POS="150,129"
  HOME_INNER_RADIUS=2
  HOME_OUTER_RADIUS=5
  SURVEY_SPEED=1.6
  SIMULATION=TRUE
  BHV_FILE=targ_zboat_lines.bhv
fi

if [ $SCENARIO = 3 ]; then
  #MOOS Stuff
  START_POS="150,129"
  LAT_ORIGIN=36.8525995
  LONG_ORIGIN=-76.3003448
  TIFF_FILE=data/Norfolk_Chart.tif
  TIFF_FILE_B=data/Norfolk_Aerial.tif
  PAN_X=-322
  PAN_Y=-284
  ZOOM=1.4
  #In this case, the shore VM
  SHORE_IP="73.219.106.147"
  SHORE_PORT=9300
  BOAT_MODEM_IP="166.150.167.176"
  BOAT_PORT=9301
  #Behavior
  HOME_POS="150,129"
  HOME_INNER_RADIUS=2
  HOME_OUTER_RADIUS=5
  SURVEY_SPEED=1.6
  SIMULATION=FALSE
  BHV_FILE=targ_zboat_lines.bhv
fi

if [ $SCENARIO = 4 ]; then
  #MOOS Stuff
  START_POS="x=0,y=0,speed=0,heading=180"
  LAT_ORIGIN=36.920164
  LONG_ORIGIN=-76.144672
  TIFF_FILE=data/LakeBradfordAerial.tif
  TIFF_FILE_B=data/LakeBradfordAerial.tif
  PAN_X=980
  PAN_Y=-2100
  ZOOM=0.72
  #In this case, the shore VM
  #SHORE_IP="192.168.1.26"
  SHORE_IP="73.219.106.147"
  SHORE_PORT=9300
  #In this case, the RPi
  #BOAT_MODEM_IP="192.168.1.243"  #26 for laptop, 243 for RPi
  BOAT_MODEM_IP="166.161.71.97"
  BOAT_PORT=9301
  #Behavior
  HOME_POS="20,0"
  HOME_INNER_RADIUS=2
  HOME_OUTER_RADIUS=5
  SURVEY_SPEED=1.6
  TEST_SPEED=1.6
  SIMULATION=TRUE
  #BHV_FILE=targ_consthead_pattern.bhv
fi  
# What is nsplug? Type "nsplug --help" or "nsplug --manual"

#nsplug zboat_lines_remote.moos targ_zboat_lines_remote.moos -f WARP=$TIME_WARP \
#   LAT_ORIGIN=$LAT_ORIGIN   LONG_ORIGIN=$LONG_ORIGIN   START_POS=$START_POS \
#   SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP  SHORE_PORT=$SHORE_PORT \
#   BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  BHV_FILE=$BHV_FILE



#nsplug zboat_statichead.bhv targ_zboat_statichead.bhv -f HOME_POS=$HOME_POS \
#  SURVEY_SPEED=$SURVEY_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
#  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

#===================================================
# New Style meta files
#===================================================

# Mission Configs
#---------------------------------------------------

nsplug meta_test_mras.moos targ_test_mras.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BHV_FILE=targ_consthead_pattern.bhv

nsplug meta_manual_control.moos targ_manual_control.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION

nsplug meta_zboat_lines.moos targ_zboat_lines.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BHV_FILE=targ_zboat_lines.bhv

nsplug meta_zboat_lines.moos targ_circle_pattern.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN  LONG_ORIGIN=$LONG_ORIGIN  \
   START_POS=$START_POS  SHORE_IP=$SHORE_IP   BOAT_MODEM_IP=$BOAT_MODEM_IP \
   SHORE_PORT=$SHORE_PORT BOAT_PORT=$BOAT_PORT  SIMULATION=$SIMULATION  \
   BHV_FILE=targ_circle_pattern.bhv

nsplug meta_laptop.moos targ_laptop.moos --path=../shared_plugins \
   -f WARP=$TIME_WARP  LAT_ORIGIN=$LAT_ORIGIN   LONG_ORIGIN=$LONG_ORIGIN \
   TIFF_FILE="$TIFF_FILE"  PAN_X=$PAN_X  PAN_Y=$PAN_Y  ZOOM=$ZOOM \
   SHORE_IP=$SHORE_IP  SHORE_PORT=$SHORE_PORT  TIFF_FILE_B=$TIFF_FILE_B \
   START_POS=$START_POS

# Behaviors
#---------------------------------------------------

nsplug meta_consthead_pattern.bhv targ_consthead_pattern.bhv -f HOME_POS=$HOME_POS \
  SURVEY_SPEED=$SURVEY_SPEED TEST_SPEED=$TEST_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

nsplug meta_zboat_lines.bhv targ_zboat_lines.bhv --path=../shared_plugins/behavior \
 -f HOME_POS=$HOME_POS \
  SURVEY_SPEED=$SURVEY_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

nsplug meta_circle_pattern.bhv targ_circle_pattern.bhv --path=../shared_plugins/behavior \
 -f HOME_POS=$HOME_POS TEST_SPEED=$TEST_SPEED \
  SURVEY_SPEED=$SURVEY_SPEED HOME_INNER_RADIUS=$HOME_INNER_RADIUS \
  HOME_OUTER_RADIUS=$HOME_OUTER_RADIUS

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
# printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
# pAntler targ_$VNAME1.moos >& /dev/null &
# sleep .5
# printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
# pAntler targ_$VNAME2.moos >& /dev/null &
# sleep .5
# printf "Launching $VNAME_ASV MOOS Community (WARP=%s) \n" $TIME_WARP
# pAntler targ_asv.moos >& /dev/null &
# sleep .5
# printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
# pAntler targ_shoreside.moos >& /dev/null &
# printf "Done \n"

# uMAC targ_shoreside.moos

# printf "Killing all processes ... \n"
# kill %1 %2 %3 %4
# printf "Done killing processes.   \n"
