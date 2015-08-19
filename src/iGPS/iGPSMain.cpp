// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/iGPSMain.cpp,v 5.1 2005/04/27 20:41:40 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

#include "GPSInstrument.h"

int main(int argc , char * argv[])
{
	const char * sMissionFile = "Mission.moos";

	if (argc > 1) {
		sMissionFile = argv[1];
	}

	CGPSInstrument GPSInstrument;

	GPSInstrument.Run("iGPS", sMissionFile);


	return 0;
}
