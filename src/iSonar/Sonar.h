// Sonar.h: interface for the Sonar interface class.
//
//////////////////////////////////////////////////////////////////////

//This is a hack since apparently Clang doesn't define this
#define UNIX

#include "NMEAMessage.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Comms/XPCUdpSocket.h"

#include <iostream>
#include <math.h>

enum class InputMode {
	Serial,
	UDP,
};

class Sonar : public CMOOSInstrument
{

	public:
		Sonar();
		virtual ~Sonar();

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

		void ProcessPacket(char* pUdpPacket);
		bool SetupUDPPort();
		string m_sType;
		unsigned int m_iUDPPort;
		bool m_bIgnoreNumSats;
		XPCUdpSocket* m_pListenSocket;

		//Configuration params
		InputMode m_mode;
		double m_transducer_depth;

};
