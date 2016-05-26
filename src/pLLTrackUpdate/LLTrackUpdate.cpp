/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH / NOAA                                      */
/*    FILE: LLTrackUpdate.cpp                               */
/*    DATE: 25 Aug 2015                                     */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "LLTrackUpdate.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsPoly.h"
// #include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
// #include "XYSegList.h"

using namespace std;

//---------------------------------------------------------
// Constructor

LLTrackUpdate::LLTrackUpdate()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_survey_update = false;
  m_turn_update = false;
  m_alignment_update = false;
}

//---------------------------------------------------------
// Destructor

LLTrackUpdate::~LLTrackUpdate()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool LLTrackUpdate::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    if (msg.GetKey() == "LLTRACK_SURVEYLINE") {
      //process the lat/long to UTM
    } else if (msg.GetKey() == "LLTRACK_TURNPOINT") {

    } else if (msg.GetKey() == "LLTRACK_ALIGNMENTLINE") {

    } else if (msg.GetKey() == "UTM_SURVEYLINE") {
      //process the UTM to local lines
      string msg_content = msg.GetString();
      m_survey_pts = ProcessUTMString(msg_content);
      if (m_survey_pts.size() != 0) {
        m_survey_update = true;
      }
    } else if (msg.GetKey() == "UTM_TURNPOINT") {
      string msg_content = msg.GetString();
      m_turn_pts = ProcessUTMString(msg_content);
      if (m_turn_pts.size() != 0) {
        m_turn_update = true;
      }
    } else if (msg.GetKey() == "UTM_ALIGNMENTLINE") {
      string msg_content = msg.GetString();
      m_alignment_pts = ProcessUTMString(msg_content);
      if (m_alignment_pts.size() != 0) {
        m_alignment_update = true;
      }
    }

#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool LLTrackUpdate::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);

   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool LLTrackUpdate::Iterate()
{
  m_iterations++;
  double dfTimeNow = MOOSTime();

  // Post the point updates if needed.
  if (m_survey_update) {
    string points_post = "points=";
    //Get points with one decimal precision
    points_post += m_survey_pts.get_spec(1);
    m_Comms.Notify("SURVEY_UPDATE", points_post, dfTimeNow);
    m_survey_update = false;
  }
  if (m_turn_update) {
    string points_post = "points=";
    points_post += m_turn_pts.get_spec(1);
    m_Comms.Notify("TURN_UPDATE", points_post, dfTimeNow);
    m_turn_update = false;
  }
  if (m_alignment_update) {
    string points_post = "points=";
    points_post += m_alignment_pts.get_spec(1);
    m_Comms.Notify("START_UPDATE", points_post, dfTimeNow);
    m_alignment_update = false;
  }

  MOOSTrace("Origin UTM: %0.2f, %0.2f\n", m_Geodesy.GetOriginEasting(),
    m_Geodesy.GetOriginNorthing());
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool LLTrackUpdate::OnStartUp()
{
  // Setup the Geodesy engine
  double dfLatOrigin;
  double dfLongOrigin;

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

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);

      if(param == "FOO") {
        //handled
      }
      else if(param == "BAR") {
        //handled
      }
    }
  }

  m_timewarp = GetMOOSTimeWarp();

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void LLTrackUpdate::RegisterVariables()
{
  // Register("FOOBAR", 0);
  Register("LLTRACK_SURVEYLINE", 0);
  Register("LLTRACK_TURNPOINT", 0);
  Register("LLTRACK_ALIGNMENTLINE", 0);
  Register("UTM_SURVEYLINE", 0);
  Register("UTM_TURNPOINT", 0);
  Register("UTM_ALIGNMENTLINE", 0);
}

XYSegList LLTrackUpdate::ProcessUTMString(string utm_points_msg) {
  XYSegList input_points = PointsString2SegList(utm_points_msg);
  if(input_points.size() == 0) {
    return input_points;
  }
  //Shift to the orgin od the local grid
  double x_shift = -m_Geodesy.GetOriginEasting();
  double y_shift = -m_Geodesy.GetOriginNorthing();
  input_points.shift_horz(x_shift);
  input_points.shift_vert(y_shift);
  return input_points;
}

XYSegList LLTrackUpdate::PointsString2SegList (string points_msg) {
  string param = biteStringX(points_msg, '=');
  XYSegList new_seglist;
  // Translate the polygon or point string to a SegList
  // String is in standard format: points=0,0:100,10
  if((param == "polygon") || (param == "points")) {
    new_seglist = string2SegList(points_msg);
    if(new_seglist.size() == 0) {
      XYPolygon new_poly = string2Poly(points_msg);
      //The logic for first point is here as a placeholder (0,0)
      new_seglist = new_poly.exportSegList(0, 0);
    }
  }
  else if(param == "point") {
    XYPoint point = string2Point(points_msg);
    new_seglist.add_vertex(point);
  }
  return new_seglist;
}
