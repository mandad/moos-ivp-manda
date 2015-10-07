#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_build, -j    \n" 
	printf "  --help, -h         \n" 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
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
VNAME1="boat1"      # The first   vehicle community
VNAME2="boat2"       # The second  vehicle community
VNAME_ASV="ASV"
START_POS1="500,500"  
START_POS2="760,-1180"
START_POS_ASV="0,0"


# What is nsplug? Type "nsplug --help" or "nsplug --manual"

nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside"  

nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  VTYPE=SHIP \
   VNAME=$VNAME1      START_POS=$START_POS1  VLENGTH=15                   \
   VPORT="9001"       SHARE_LISTEN="9301"    

nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1 START_POS=$START_POS1 

nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP  VTYPE=SHIP \
   VNAME=$VNAME2      START_POS=$START_POS2  VLENGTH=15                  \
   VPORT="9002"       SHARE_LISTEN="9302"    

nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2 START_POS=$START_POS2

nsplug meta_asv.moos targ_asv.moos -f WARP=$TIME_WARP  VTYPE=SHIP \
   VNAME=$VNAME_ASV     START_POS=$START_POS_ASV   VLENGTH=4             \
   VPORT="9003"       SHARE_LISTEN="9303"  

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME1.moos >& /dev/null &
sleep .5
printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME2.moos >& /dev/null &
sleep .5
printf "Launching $VNAME_ASV MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_asv.moos >& /dev/null &
sleep .5
printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"


