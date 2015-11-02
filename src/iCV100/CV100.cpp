// $Header: /raid/cvs-server/REPOSITORY/software/MOOS/interface/general/iGPS/CV100.cpp,v 5.18 2007/01/31 03:53:10 anrp Exp $
// copyright (2001-2003) Massachusetts Institute of Technology (pnewman et al.)

// CV100.cpp: implementation of the CCV100 class.
//
//////////////////////////////////////////////////////////////////////
#include <cstring>
#include "CV100.h"

using namespace std;



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCV100::CCV100()
{
  m_sType = "SERIAL";
  m_dfSimDepth = 15;
  m_bSimulateMode = false;
}

CCV100::~CCV100()
{
}


/////////////////////////////////////////////
///this is where it all happens..
bool CCV100::Iterate()
{
	if (GetData()) {
		PublishData();
	}

	return true;
}


bool CCV100::OnStartUp()
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

  string sSimMode;
  if (m_MissionReader.GetConfigurationParam("Simulate", sSimMode)) {
    MOOSTrace("sSimMode=\""+sSimMode+"\"\n");
    if (sSimMode == "true") {
      MOOSTrace("Setting Sim Mode\n");
      m_bSimulateMode = true;
    }
  }

  string sDepthConfig;
  if (m_MissionReader.GetConfigurationParam("SimDepth", sDepthConfig)) {
    m_dfSimDepth = atof(sDepthConfig.c_str());
  } else {
    if (IsSimulateMode()) {
      MOOSTrace("Warning: Simulate mode set, but no simulate depth configured.\n");
    }
  }



	//here we make the variables that we are managing
	double dfPeriod = 0;

	//GPS update @ 2Hz
	// AddMOOSVariable("X", "SIM_X", "GPS_X", dfPeriod);

	// AddMOOSVariable("Y", "SIM_Y", "GPS_Y", dfPeriod);


	if (IsSimulateMode()) {
		//not much to do...
		RegisterMOOSVariables();
	} else {
		//try to open

		if (!SetupPort()) {
			return false;
		}

		//try 10 times to initialise sensor
		if (!InitialiseSensorN(10, "CV100")) {
			return false;
		}
	}


	return true;
}

bool CCV100::IsSimulateMode() {
  return m_bSimulateMode;
}

bool CCV100::OnNewMail(MOOSMSG_LIST &NewMail)
{
	return UpdateMOOSVariables(NewMail);
}

bool CCV100::PublishData()
{
	return PublishFreshMOOSVariables();
}



bool CCV100::OnConnectToServer()
{
	if (IsSimulateMode()) {
		//not much to do...
		RegisterMOOSVariables();
	} else {

	}

	return true;
}


///////////////////////////////////////////////////////////////////////////
// here we initialise the sensor, giving it start up values
bool CCV100::InitialiseSensor()
{
	return true;
}



/**
*
*/

bool CCV100::GetData()
{
	if (!IsSimulateMode()) {
		//here we actually access serial ports etc

		string sWhat;

		double dfWhen;

		if (m_Port.IsStreaming()) {
			if (!m_Port.GetLatest(sWhat, dfWhen)) {
				return false;
			}
		} else {
			if (!m_Port.GetTelegram(sWhat, 0.5)) {
				return false;
			}
		}

		//MOOSTrace("Rx:  %s",sWhat.c_str());
		if (PublishRaw()) {
			SetMOOSVar("Raw", sWhat, MOOSTime());
		}

		ParseNMEAString(sWhat);

	} else {
    // Post a constant depth
    m_Comms.Notify("SONAR_DEPTH_M", m_dfSimDepth);
	}

	return true;

}


bool CCV100::ParseNMEAString(string &sNMEAString)
{
  //keep a copy for later..
  string sCopy = sNMEAString;
  string sWhat = MOOSChomp(sNMEAString, ",");
  bool bGood = true;
  
    //first of all is this a good NMEA string?
  if (!DoNMEACheckSum(sCopy)) {
    MOOSTrace("CV100 Failed NMEA check sum");
    return false;
  }

  if (sWhat == "$SDDBS") {
    NMEAMessage m(sCopy);
    //$SDDBS,0.0,f,0.00,M,0.0,F*31

    //Depth in meters
    if(strlen(m.Part(3).c_str())) {
        m_Comms.Notify("SONAR_DEPTH_M", atof(m.Part(3).c_str()));
    }
    
    return true;
  }

  return false;
}
