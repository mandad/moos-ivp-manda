#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=25
SCENARIO=8
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

# 1 = Summer Hydro 2015 half step test
# 2 = Strait of Georgia 
# 3 = Fairweather H12758 Offshore
# 4 = H12472 Shumagins
# 5 = Summer Hydro 2015 North Region
# 6 = SH 15 South Region
# 7 = H12450 Chirikof
# 8 = H12898 S Kodiak Estimate
if [ $SCENARIO = 1 ]; then
  #MOOS Stuff
  START_POS="x=705,y=-4379,speed=0,heading=180"
  LAT_ORIGIN=42.9185596
  LONG_ORIGIN=-70.7960903
  TIFF_FILE=SH_2015.tif
  PAN_X=814 #258
  PAN_Y=769 #927
  ZOOM=1.98 #0.53
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto
  OP_WKT="POLYGON_((655_-4429,550_-4813,198_-4725,300_-4353))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/SH15_Surface.tif'"
  X_OFFSET=353408.656
  Y_OFFSET=6083.832+4753335.914
elif [ $SCENARIO = 2 ]; then
  #MOOS Stuff
  # START_POS="4082,9023"
  START_POS="x=4998,y=13971,speed=0,heading=90"
  LAT_ORIGIN=48.7188051
  LONG_ORIGIN=-122.8272363
  TIFF_FILE=H12322_8m_Color.tif
  PAN_X=30
  PAN_Y=-593
  ZOOM=2.22
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto #"4948,14031:3916,14613"
  # Python Stuff
  # OP_POLY="[(4032,9073),(2838,12080),(5720,13208),(7249,10238)]"
  OP_POLY="[(4948,14031),(3916,14613),(5226,16027),(5579,15694),(5954,15515)]"
  OP_WKT="POLYGON_((4948_14031,3916_14613,5226_16027,5579_15694,5954_15515))"
  BATHY_GRID="'../path_planning/terrain/H12322_8m_Depths.tiff'"
  X_OFFSET=512684.0
  Y_OFFSET=5396228.0
elif [ $SCENARIO = 3 ]; then
  #MOOS Stuff
  START_POS="x=1313,y=10426,speed=0,heading=180"
  LAT_ORIGIN=55.1066916
  LONG_ORIGIN=-159.3599676
  TIFF_FILE=FA_Polygon1_Color_Tracklines.tif
  PAN_X=243
  PAN_Y=-7
  ZOOM=0.14
  FIRST_SIDE=Port
  FIRST_LINE="1339,10190:1422,4698:1072,1377"
  OP_WKT="POLYGON_((1339_10190,1422_4698,1072_1377,5245_1246,5049_6155,4361_10249))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/FA_Polygon1_Depths.tif'"
  X_OFFSET=477063.099
  Y_OFFSET=6106708.425
elif [ $SCENARIO = 4 ]; then
  #MOOS Stuff
  START_POS="x=570,y=8535,speed=0,heading=90"
  LAT_ORIGIN=54.9103441
  LONG_ORIGIN=-159.5186646
  TIFF_FILE=H12472_4m_Color.tif
  PAN_X=1344
  PAN_Y=-732
  ZOOM=1.25
  FIRST_SIDE=Port
  FIRST_LINE="643,8535:2245,8535"
  OP_WKT="POLYGON_((643_8535,2245_8535,3898_10920,1994_10920,1777_12134,643_11609))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/H12472_4m_Depths.tiff'"
  X_OFFSET=466802.000
  Y_OFFSET=6084894.000
elif [ $SCENARIO = 5 ]; then
  #MOOS Stuff
  # START_POS="x=4107,y=-589,speed=0,heading=225"
  START_POS="x=3758,y=4972,speed=0,heading=225"
  LAT_ORIGIN=42.9185596
  LONG_ORIGIN=-70.7960903
  TIFF_FILE=SH_2015.tif
  PAN_X=-535
  PAN_Y=-788
  ZOOM=0.60
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto
  #OP_WKT="POLYGON_((4075_-650,3293_-2464,2405_-2259,3180_-387))"
  OP_WKT="POLYGON_((3739_4927,3232_3763,2341_3956,2910_5317))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/SH15_Surface.tif'"
  X_OFFSET=353408.656
  Y_OFFSET=4753335.914
elif [ $SCENARIO = 6 ]; then
  #MOOS Stuff
  START_POS="x=2454,y=1899,speed=0,heading=225"
  LAT_ORIGIN=42.9185596
  LONG_ORIGIN=-70.7960903
  TIFF_FILE=SH_2015.tif
  PAN_X=258
  PAN_Y=927
  ZOOM=0.53
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto
  OP_WKT="POLYGON_((2422_1825,1666_147,500_768,1214_2374))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/SH15_Surface.tif'"
  X_OFFSET=353408.656
  Y_OFFSET=4753335.914
elif [ $SCENARIO = 7 ]; then
  #MOOS Stuff
  START_POS="x=8079,y=10357,speed=0,heading=315"
  LAT_ORIGIN=55.8403068
  LONG_ORIGIN=-155.6171624
  TIFF_FILE=H12450_4m_Color.tif
  PAN_X=-663
  PAN_Y=-992
  ZOOM=0.57
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto
  OP_WKT="POLYGON_((8002_10463,5837_13928,6195_13928,6547_13996,6848_14083,7099_14238,7229_14479,8707_12208,8662_10841))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/H12450_4m_Depths.tiff'"
  # X_OFFSET=336422.000
  # Y_OFFSET=6191214.000
  X_OFFSET=336975.000
  Y_OFFSET=6190995.000
elif [ $SCENARIO = 8 ]; then
  #MOOS Stuff
  START_POS="x=27327,y=-7660,speed=0,heading=225"
  LAT_ORIGIN=56.9294949
  LONG_ORIGIN=-153.8841530
  TIFF_FILE=OPR_P335_View.tif
  PAN_X=-55
  PAN_Y=26
  ZOOM=0.30
  FIRST_SIDE=Stbd
  FIRST_LINE=Auto
  OP_WKT="POLYGON_((26697_-8133,10595_-20057,6734_-14830,9544_-13333,9334_-12676,23230_-2223))"
  # Python Stuff
  BATHY_GRID="'../path_planning/terrain/OPR_P335_Prior_Depth.tiff'"
  #X_OFFSET=336975.000
  #Y_OFFSET=6190995.000
fi
# What is nsplug? Type "nsplug --help" or "nsplug --manual" 

nsplug sonar_sim.moos targ_sonar_sim.moos --path=../shared_plugins -f WARP=$TIME_WARP \
   LAT_ORIGIN=$LAT_ORIGIN   LONG_ORIGIN=$LONG_ORIGIN   START_POS=$START_POS \
   TIFF_FILE="$TIFF_FILE"  PAN_X=$PAN_X  PAN_Y=$PAN_Y  ZOOM=$ZOOM  \
   OP_WKT="$OP_WKT" FIRST_SIDE=$FIRST_SIDE  FIRST_LINE=$FIRST_LINE

# nsplug ~/code/asv-dev/utilities/python_moosapps/path_plan.py \
#   ~/code/asv-dev/utilities/python_moosapps/targ_path_plan.py -f \
#   OP_POLY=$OP_POLY  WARP=$TIME_WARP

nsplug ~/code/asv-dev/utilities/python_moosapps/sonar_simulator.py \
  ~/code/asv-dev/utilities/python_moosapps/targ_sonar_simulator.py -f \
  BATHY_GRID=$BATHY_GRID  X_OFFSET=$X_OFFSET \
  Y_OFFSET=$Y_OFFSET WARP=$TIME_WARP

# nsplug ~/code/asv-dev/utilities/python_moosapps/record_swath.py \
#   ~/code/asv-dev/utilities/python_moosapps/targ_record_swath.py -f \
#   WARP=$TIME_WARP

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


