// Sonar.cpp: implementation of the Sonar class.
// 
// Receives sonar data passed from either serial or UDP ports
//////////////////////////////////////////////////////////////////////
#include <cstring>
#include <string>
#include <chrono>
#include <iostream>
#include <time.h>

#include "Sonar.h"

using namespace std;

#define DEBUG true

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Sonar::Sonar() : m_mode{InputMode::Serial}, m_transducer_depth{0}, 
                 m_use_utc_log_names{true}
{
	m_sType = "VANILLA";
}

Sonar::~Sonar()
{
    std::cout << "iSonar Destructor called" << endl;
    CloseFiles();
    //m_Port.Close();
}


bool Sonar::Iterate()
{
    #if DEBUG
    MOOSTrace("Iterate called\n");
    #endif
	if (GetData()) {
		PublishData();
	}

    if (m_log_hypack) {
        LogHypack();
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

    //----- Optional Config Values ----------
    if (m_MissionReader.GetValue("TranducerDepth", sVal)) {
        m_transducer_depth = std::stod(sVal);
    }

    if (m_MissionReader.GetValue("LogHypack", sVal)) {
        m_log_hypack = MOOSStrCmp(sVal, "TRUE");
        if (m_log_hypack) {
            string log_name = MakeLogName("SonarData_") + ".RAW";
            MOOSTrace("iSonar Logging sonar data to: " + log_name + "\n");
            if (OpenFile(m_hypack_log_file, log_name, false)) {
                LogHeader();
            } else {
                MOOSTrace("iSonar Error: Could not open log file for writing.\n");
                m_log_hypack = false;
            }
        }
    }


	//here we make the variables that we are managing
    //update @ 10Hz
	double dfUpdatePeriod = 0.01;

	AddMOOSVariable("X", "NAV_X", "", dfUpdatePeriod);
	AddMOOSVariable("Y", "NAV_X", "", dfUpdatePeriod);
    AddMOOSVariable("Heading", "NAV_HEADING", "", dfUpdatePeriod);

	AddMOOSVariable("Lat", "NAV_LAT", "", dfUpdatePeriod);
	AddMOOSVariable("Lon", "NAV_LON", "", dfUpdatePeriod);

    AddMOOSVariable("Depth", "", "SONAR_DEPTH_M", 0);
    AddMOOSVariable("Raw", "", "SONAR_RAW", 0);

    //try to open port
    if (m_mode == InputMode::Serial) {
        if (!SetupPort()) {
            MOOSTrace("iSonar Error: Could not open serial port.\n");
            return false;
        }
    } else if (m_mode == InputMode::UDP) {
    	if (!SetupUDPPort()) {
            MOOSTrace("iSonar Error: Could not open UDP port.\n");
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
    //Parses data from network with just strings in the data
    #if DEBUG
        MOOSTrace("Sonar Packet Received:");
        MOOSTrace(pUdpPacket);
    #endif
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

//=========== Logging Functions =============================

bool Sonar::LogHypack() {
    if (!m_hypack_log_file.is_open()) {
        return false;
    }
    auto curr_x = GetMOOSVar("X");
    auto curr_y = GetMOOSVar("Y");
    auto sonar_depth = GetMOOSVar("Depth");
    if ((curr_y->IsFresh() || curr_x->IsFresh()) && sonar_depth->IsFresh()) {
        double UTM_x = curr_x->GetDoubleVal() + m_Geodesy.GetOriginEasting();
        double UTM_y = curr_y->GetDoubleVal() + m_Geodesy.GetOriginNorthing();
        double timestamp = SecondsPastMidnight();
        auto heading = GetMOOSVar("Heading");
        m_hypack_log_file << std::fixed << std::setprecision(3);
        m_hypack_log_file << "POS 1 " << timestamp << " " << UTM_x << " " << UTM_y << "\n";
        m_hypack_log_file << "GYR 1 " << timestamp << " " << heading->GetDoubleVal() << "\n";
        m_hypack_log_file << "EC1 0 " << timestamp << " " << sonar_depth->GetDoubleVal() << endl        ;
    }

    return true;
}

void Sonar::LogHeader() {
    m_hypack_log_file << "FTP NEW 2\n";
    m_hypack_log_file << "VER 15.0.9.71\n";
    m_hypack_log_file << "INF \"Manda\" \"ASV\" \"Project\" 0.000000 0.000000 1500.000000\n";
    m_hypack_log_file << "DEV 0 16 \"CEEPULSE 100\"\n";
    m_hypack_log_file << "OFF 0 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\n";
    m_hypack_log_file << "DEV 1 100 \"GP9 GPS/IMU\"\n";
    m_hypack_log_file << "OFF 0 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\n";
    m_hypack_log_file << "EOH" << endl;
}

double Sonar::SecondsPastMidnight() {
    // The C++ way
    // auto now = std::chrono::system_clock::now();

    // time_t tnow = std::chrono::system_clock::to_time_t(now);
    // tm *date = std::localtime(&tnow);
    // date->tm_hour = 0;
    // date->tm_min = 0;
    // date->tm_sec = 0;
    // auto midnight = std::chrono::system_clock::from_time_t(std::mktime(date));

    // std::chrono::system_clock::duration duration_past = now-midnight;

    time_t t1, t2;
    struct tm tms;
    time(&t1);
    gmtime_r(&t1, &tms);
    tms.tm_hour = 0;
    tms.tm_min = 0;
    tms.tm_sec = 0;
    t2 = mktime(&tms);
    return t1 - t2;
}

// =========  From MOOSLogger.cpp (pLogger)  ================
std::string Sonar::MakeLogName(string sStem)
{
    struct tm *Now;
    time_t aclock;
    time( &aclock );

    // We should probably always use UTC
    if(m_use_utc_log_names)
    {
        Now = gmtime(&aclock);
    }
    else
    {
        Now = localtime( &aclock );
    }

    std::string  sTmp;

    // Always append the timestamp
    if(true)
    {
        // Print local time as a string

        sTmp = MOOSFormat( "%s_%d_%02d_%02d__%.2d_%.2d_%.2d",
            sStem.c_str(),
            Now->tm_year+1900,
            Now->tm_mon+1,
            Now->tm_mday,
            Now->tm_hour,
            Now->tm_min,
            Now->tm_sec);
    }
    else
    {
        sTmp = MOOSFormat("%s",sStem.c_str());
    }

    return sTmp;

}

bool Sonar::OpenFile(std::ofstream & of, const std::string & sName, bool bBinary)
{
    if(!bBinary)
        of.open(sName.c_str());
    else
    {
        of.open(sName.c_str(), std::ios::binary);
    }


    if(!of.is_open())
    {
        string sErr = MOOSFormat("ERROR: Failed to open File: %s",sName.c_str());
        MOOSDebugWrite(sErr);
        return false;
    }

    return true;
}

bool Sonar::CloseFiles()
{
    if(m_hypack_log_file.is_open())
    {
        MOOSTrace("iSonar Closing Log File.\n");
        m_hypack_log_file << std::endl;
        m_hypack_log_file.close();
    }
    return true;
}
