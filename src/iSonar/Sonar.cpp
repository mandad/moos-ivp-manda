// Sonar.cpp: implementation of the Sonar class.
// 
// Receives sonar data passed from either serial or UDP ports
//////////////////////////////////////////////////////////////////////
#include <cstring>
#include <string>
#include "Sonar.h"

using namespace std;

#define DEBUG false

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Sonar::Sonar() : m_mode{InputMode::Serial}, m_transducer_depth{0}
{
	m_sType = "VANILLA";
}

Sonar::~Sonar()
{
}


/////////////////////////////////////////////
///this is where it all happens..
bool Sonar::Iterate()
{
	if (GetData()) {
		PublishData();
	}

	return true;
}


bool Sonar::OnStartUp()
{
	CMOOSInstrument::OnStartUp();

	//set up Geodetic conversions
	double dfLatOrigin;
	double dfLongOrigin;


	m_MissionReader.GetConfigurationParam("TYPE", m_sType);

	string sVal;

	if (m_MissionReader.GetValue("LatOrigin", sVal)) {
		dfLatOrigin = atof(sVal.c_str());
	} else {
		MOOSTrace("LatOrigin not set - FAIL\n");

		return false;

	}

	if (m_MissionReader.GetValue("LongOrigin", sVal)) {
		dfLongOrigin = atof(sVal.c_str());
	} else {
		MOOSTrace("LongOrigin not set - FAIL\n");

		return false;
	}

	if (!m_Geodesy.Initialise(dfLatOrigin, dfLongOrigin)) {
		MOOSTrace("Geodesy Init failed - FAIL\n");

		return false;
	}


    std::string mode;
    if (!m_MissionReader.GetConfigurationParam("Mode", mode)) {
        MOOSTrace("Mode not set - Defaulting to serial\n");
    } else {
        if (mode.compare("Serial") == 0)  {
            m_mode = InputMode::Serial;
        } else if (mode.compare("UDP") == 0) {
            m_mode = InputMode::UDP;
        }
    }

    // Serial is read in by CMOOSInstrument::OnStartUp()
    // Note port is reused for either serial or network
    if (m_mode == InputMode::UDP) {
        //port = 56004
        if (!m_MissionReader.GetConfigurationParam("Port", m_iUDPPort)) {
            MOOSTrace("Port not set - FAIL\n");
            return false;
        }
    }

    if (m_MissionReader.GetValue("TranducerDepth", sVal)) {
        m_transducer_depth = atof(sVal.c_str());
    }

	//here we make the variables that we are managing
    //update @ 10Hz
	double dfUpdatePeriod = 0.01;

	AddMOOSVariable("X", "NAV_X", "", dfUpdatePeriod);
	AddMOOSVariable("Y", "NAV_X", "", dfUpdatePeriod);

	AddMOOSVariable("Lat", "NAV_LAT", "", dfUpdatePeriod);
	AddMOOSVariable("Lon", "NAV_LON", "", dfUpdatePeriod);

    AddMOOSVariable("Depth", "", "SONAR_DEPTH_M", 0);
    AddMOOSVariable("Raw", "", "SONAR_RAW", 0);

    //try to open port
    if (m_mode == InputMode::Serial) {
        if (!SetupPort()) {
            return false;
        }
    } else if (m_mode == InputMode::UDP) {
    	if (!SetupUDPPort()) {
    		return false;
    	}
    }

	//try 10 times to initialise sensor
	if (!InitialiseSensorN(10, "Sonar")) {
		return false;
	}


	return true;
}

bool Sonar::SetupUDPPort()
{
    m_pListenSocket = new XPCUdpSocket((long)m_iUDPPort);
    m_pListenSocket->vBindSocket();
    return true;
}



bool Sonar::OnNewMail(MOOSMSG_LIST &NewMail)
{
	return UpdateMOOSVariables(NewMail);
}

bool Sonar::PublishData()
{
	return PublishFreshMOOSVariables();
}



bool Sonar::OnConnectToServer()
{
	RegisterMOOSVariables();

	return true;
}


///////////////////////////////////////////////////////////////////////////
// here we initialise the sensor, giving it start up values
bool Sonar::InitialiseSensor()
{
    //We don't have any need to initialize the RTA

	return true;

}



/**
*
*/

bool Sonar::GetData()
{
    	//here we actually access serial ports etc

    if (m_mode == InputMode::Serial) {
    	string sWhat;
    	double dfWhen;

	#if DEBUG
	MOOSTrace("Getting Serial Data\n");
	#endif
    	if (m_Port.IsStreaming()) {
    		if (!m_Port.GetLatest(sWhat, dfWhen)) {
    			//return false;
    		}
    	} else {
    		if (!m_Port.GetTelegram(sWhat, 0.5)) {
    			//return false;
    		}
    	}
	
        if (PublishRaw()) {
            SetMOOSVar("Raw", sWhat, MOOSTime());
        }
	#if DEBUG
	MOOSTrace("Parsing Data: " + sWhat + "\n");
	#endif
        ParseNMEAString(sWhat);
    } else if (m_mode == InputMode::UDP) {
        char buffer[1472];  //traditional max for 1500 MTU
        try
        {
            if (DEBUG)
                MOOSTrace("Receiving message on UDP port\n");
            if (m_pListenSocket->iRecieveMessage(buffer, sizeof(buffer), 0)
                == sizeof(buffer)) {
                MOOSTrace("Packet Overflows Buffer");
            } else {
                ProcessPacket(buffer);
            }
        } catch (int e) {
            MOOSTrace("Error Receiving Data from GPS\n");
            return false;
        }

    	//MOOSTrace("Rx:  %s",sWhat.c_str());
    	if (PublishRaw()) {
    		SetMOOSVar("Raw", buffer, MOOSTime());
    	}
    }

    return true;

}

void Sonar::ProcessPacket(char* pUdpPacket)
{
    const bool HypackMode = true;

    if (HypackMode) {
        //Parses data from network with just strings in the data
        if (DEBUG) {
            MOOSTrace("Sonar Packet Received:");
            MOOSTrace(pUdpPacket);
        }
        // There can potentially be multiple messages per ethernet packet
        char * pThisMessage;
        pThisMessage = strtok(pUdpPacket, "\r\n");
        while (pThisMessage != NULL) {
            string sThisMessage(pThisMessage);

            if (!ParseNMEAString(sThisMessage)) {
                if (DEBUG)
                    MOOSTrace("Unable to process NMEA string.");
            }
            pThisMessage = strtok(NULL, "\r\n");
        }
    } else {
        char * pNMEAMessage = &pUdpPacket[31];
        unsigned int iMsgLength = 0;
        // There could also be a sound velocity message
        // Could also uses pNMEAMessage[7] as numerical indicator
        if (strstr(pNMEAMessage, "$GPGGA") != NULL) {
            iMsgLength = 80;

        } else if (strstr(pNMEAMessage, "$GPHDT") != NULL) {
            iMsgLength = 19;
        }

        // If a message was found, process it
        if (iMsgLength > 0) {
            string sNMEAMessage(pNMEAMessage, iMsgLength);
            if (!ParseNMEAString(sNMEAMessage)) {
                if (DEBUG)
                    MOOSTrace("Unable to process NMEA string.");
            }
        } else {
            if (DEBUG) {
                MOOSTrace("No GPS data found in RTA message.");
                MOOSTrace(pNMEAMessage);
            }
        }
    }
    
}


bool Sonar::ParseNMEAString(string &sNMEAString)
{
    //keep a copy for later..
    string sCopy = sNMEAString;
    string sWhat = MOOSChomp(sNMEAString, ",");
    bool bGood = true;
  
    //first of all is this a good NMEA string?
    if (!DoNMEACheckSum(sCopy)) {
        MOOSDebugWrite("GPS Failed NMEA check sum");
        return false;
    }

    if (sWhat.compare(3, 3, "DBT") == 0) {
        // Depth below transducer (not compensated)
        NMEAMessage m(sCopy);

        if(strlen(m.Part(3).c_str())) {
            SetMOOSVar("Depth", std::stod(m.Part(3)) + m_transducer_depth, 
                MOOSTime());
        }
        return true;
    } else if (sWhat.compare(3, 3, "DBS") == 0) {
        // Depth below surface
        NMEAMessage m(sCopy);

        if(strlen(m.Part(3).c_str())) {
            SetMOOSVar("Depth", std::stod(m.Part(3)), MOOSTime());
            // m_Comms.Notify("SONAR_DEPTH_M", atof(m.Part(3).c_str()));
        }
        return true;
    }

    return false;
}
