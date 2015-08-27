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

  string sDepthConfig;
  if (!m_MissionReader.GetConfigurationParam("SimDepth", sDepthConfig)) {
    m_dfSimDepth = atof(sDepthConfig.c_str());
  } else {
    if (IsSimulateMode()) {
      MOOSTrace("Warning: Simulate mode set, but not simulate depth configured.\n");
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
		if (!InitialiseSensorN(10, "GPS")) {
			return false;
		}
	}


	return true;
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


  //GGA and GLL headers format the NMEA string differently
  
  /*
  if (sWhat == "$GPGGA") {
    
    //OK so extract data...
    double dfTimeNow = MOOSTime();
    
    string sTmp;
    
    sTmp = MOOSChomp(sNMEAString, ",");
    
    /////////////////////////////////////
    //         GPS Time
    double dfTime	= atof(	sTmp.c_str());
    m_Comms.Notify("GPS_TIME", dfTime, dfTimeNow);

    
    /////////////////////////////////////
    //          latitude..
    sTmp = MOOSChomp(sNMEAString, ",");
    if (sTmp.size() == 0)
      bGood = false;
    
    double dfLat = atof(sTmp.c_str());
    
    sTmp = MOOSChomp(sNMEAString, ",");
    
    string sNS	= sTmp;
    
    if (sNS == "S") {
      dfLat *= -1.0;
    }
    
    //////////////////////////////////////
    //          longitude
    sTmp = MOOSChomp(sNMEAString, ",");
    
    if (sTmp.size() == 0)
      bGood = false;
    
    double dfLong = atof(sTmp.c_str());
    
    sTmp = MOOSChomp(sNMEAString, ",");
    
    string sEW = sTmp;
    
    if (sEW == "W") {
      dfLong *= -1.0;
    }
    
    
    ////////////////////////////////////////
    //          quality
    
    sTmp = MOOSChomp(sNMEAString, ",");
    double dfHDOP	= atof(sTmp.c_str());
    sTmp = MOOSChomp(sNMEAString, ",");
    double dfSatellites = atof(	sTmp.c_str());
    if (dfSatellites < 4)
      bGood = false;
    
    
    /////////////////////////////////////////
    //GEODETIC CONVERSION....
    
    double dfLatDecDeg = m_Geodesy.DMS2DecDeg(dfLat);
    double dfLongDecDeg = m_Geodesy.DMS2DecDeg(dfLong);
    double dfELocal;
    double dfNLocal;
    double dfXLocal;
    double dfYLocal;
    
    if (m_Geodesy.LatLong2LocalUTM(dfLatDecDeg, dfLongDecDeg, dfNLocal, dfELocal)) {
      //set our GPS local Northings and Eastings variables
      
      if (bGood) {
	SetMOOSVar("N", dfNLocal, dfTimeNow);
	SetMOOSVar("E", dfELocal, dfTimeNow);
      }
    }
    
    if (m_Geodesy.LatLong2LocalUTM(dfLatDecDeg, dfLongDecDeg, dfYLocal, dfXLocal)) {
      //set our GPS local Grid variables
      
      if (bGood) {
	SetMOOSVar("X", dfXLocal, dfTimeNow);
	SetMOOSVar("Y", dfYLocal, dfTimeNow);
      }
    }
    
    m_Comms.Notify("GPS_LATITUDE", dfLatDecDeg);
    m_Comms.Notify("GPS_LONGITUDE", dfLongDecDeg);
    
    //always say how many satellites we have...
    SetMOOSVar("Satellites", dfSatellites, dfTimeNow);
    
    char tmp[160];
    snprintf(tmp, 160, "time=%lf,n=%lf,e=%lf,x=%lf,y=%lf,lat=%lf,lon=%lf,sat=%i,hdop=%lf",
	     dfTimeNow, dfNLocal, dfELocal, dfXLocal,
	     dfYLocal, dfLatDecDeg, dfLongDecDeg, (int)dfSatellites, dfHDOP);
    
    m_Comms.Notify("GPS_SUMMARY", tmp);
    
    return true;
  } else if(sWhat == "$GPRMC") {
    NMEAMessage m(sCopy);
    
    if(strlen(m.Part(7).c_str())) {
      m_Comms.Notify("GPS_SPEED", atof(m.Part(7).c_str()) * 0.51444444);
    }
    
    if(strlen(m.Part(8).c_str())) {
      double heading = atof(m.Part(8).c_str());
      while(heading > 180) heading -= 360;
      while(heading < -180) heading += 360;
      double yaw = -heading * M_PI/180.0;
      
      m_Comms.Notify("GPS_TRACK_ANG", heading);
      m_Comms.Notify("GPS_YAW", yaw);
    }
    
    if(strlen(m.Part(10).c_str())) {
      double f = atof(m.Part(10).c_str());
      
      if(m.Part(11)[0] == 'W') f = 0 - f;
      
      m_Comms.Notify("GPS_MAGNETIC_DECLINATION", f);
    } 
    return true;
  } else if (sWhat == "$GPHDT") {
    NMEAMessage m(sCopy);

    if(strlen(m.Part(1).c_str())) {
        m_Comms.Notify("GPS_HEADING", atof(m.Part(1).c_str()));
    }
    return true;
  } else if (sWhat == "$GPVTG") {
    NMEAMessage m(sCopy);

    //Track made good (degrees true)
    if(strlen(m.Part(1).c_str())) {
        m_Comms.Notify("GPS_VTG_HEADING", atof(m.Part(1).c_str()));
    }
    //Speed, in knots
    if(strlen(m.Part(5).c_str())) {
        m_Comms.Notify("GPS_SPEED", atof(m.Part(5).c_str()) * 0.51444444);
    } else if (strlen(m.Part(7).c_str())) {
        //Speed over ground in kilometers/hour (kph)
        m_Comms.Notify("GPS_SPEED", atof(m.Part(7).c_str()) * 0.27777778);
    }
    
    return true;
  }
  //May also want to handle ZDA
  */

  return false;
}
