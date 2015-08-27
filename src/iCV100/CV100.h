// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/CV100.h,v 5.1 2005/04/27 20:41:40 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

// CV100.h: interface for the CCV100 class.
//
//////////////////////////////////////////////////////////////////////

#include "NMEAMessage.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include <iostream>
#include <math.h>

class CCV100 : public CMOOSInstrument
{

	public:
		CCV100();
		virtual ~CCV100();

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
		bool IsSimulateMode();

	protected:
		//Config variables
		double m_dfSimDepth;
		string m_sType;
		bool m_bSimulateMode;

};
