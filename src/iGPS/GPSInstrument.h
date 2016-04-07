// GPSInstrument.h: interface for the CGPSInstrument class.
//
//////////////////////////////////////////////////////////////////////

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
