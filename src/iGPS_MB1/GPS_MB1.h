// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/GPSInstrument.h,v 5.1 2005/04/27 20:41:40 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

// GPSInstrument.h: interface for the CGPS_MB1 class.
//
//////////////////////////////////////////////////////////////////////

#include "NMEAMessage.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Comms/XPCUdpSocket.h"

#include <iostream>
#include <math.h>

class CGPS_MB1 : public CMOOSInstrument
{

	public:
		CGPS_MB1();
		virtual ~CGPS_MB1();

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
