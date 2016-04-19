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
	Replay,
};

class Sonar : public CMOOSInstrument
{

	public:
		Sonar();
		~Sonar();

	protected:
		CMOOSGeodesy m_Geodesy;
		bool ParseNMEAString(std::string & sNMEAString);
		bool InitialiseSensor();
		bool Iterate();
		bool OnNewMail(MOOSMSG_LIST &NewMail);
		bool OnConnectToServer();
		bool OnStartUp();
		bool GetData();
		bool PublishData();
		void ProcessPacket(char* pUdpPacket);
		bool SetupUDPPort();

		//Logging
		bool LogHypack();
		void LogHeader();
		std::string MakeLogName(string sStem);
		bool OpenFile(std::ofstream & of, const std::string & sName, bool bBinary);
		bool CloseFiles();
		double SecondsPastMidnight();
		std::string HypackTND();

		std::string m_sType;
		unsigned int m_iUDPPort;
		XPCUdpSocket* m_pListenSocket;
		//Logging
		std::ofstream m_hypack_log_file;
		struct tm *m_log_start;

		//Configuration params
		InputMode m_mode;
		double m_transducer_depth;
		bool m_log_hypack;
		bool m_use_utc_log_names;

};
