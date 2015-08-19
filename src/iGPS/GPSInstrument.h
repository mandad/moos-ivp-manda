// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/GPSInstrument.h,v 5.1 2005/04/27 20:41:40 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

// GPSInstrument.h: interface for the CGPSInstrument class.
//
//////////////////////////////////////////////////////////////////////

//can you blow me where the pampers is?

#include "NMEAMessage.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include <iostream>
#include <math.h>

class CGPSInstrument : public CMOOSInstrument
{

	public:
		CGPSInstrument();
		virtual ~CGPSInstrument();

	protected:
		CMOOSGeodesy m_Geodesy;
		bool ParseNMEAString(string & sNMEAString);
		bool InitialiseSensor();
		bool Iterate();
		bool OnNewMail(MOOSMSG_LIST &NewMail);
		bool OnConnectToServer();
		bool OnStartUp();
		bool GetData();
		bool PublishData();
		string m_sType;

};
