/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.cpp                                  */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RecordSwath.h"
#include "PathPlan.h"
#include "SurveyPath.h"
#include <boost/algorithm/string.hpp>

#define DEBUG true

//---------------------------------------------------------
// Constructor

SurveyPath::SurveyPath() : m_first_swath_side{BoatSide::Port},
  m_swath_interval{10}, m_alignment_line_len{10}, m_turn_pt_offset{15},
  m_remove_in_coverage{false}, m_swath_overlap{0.2}, m_line_end{false},
  m_line_begin{false}, m_turn_reached{false}, m_recording{false},
  m_swath_record(10), m_swath_side{BoatSide::Port}
{
  m_swath_side = AdvanceSide(m_first_swath_side);
  //m_swath_side = m_next_swath_side;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SurveyPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = toupper(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if(param == "OP_REGION") {
      boost::geometry::read_wkt(value, m_op_region);
    }
    else if(param == "BAR") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  // Reception variables
  AddMOOSVariable("Swath", "SWATH_WIDTH", "", 0);
  AddMOOSVariable("LineBegin", "LINE_BEGIN", "", 0);
  AddMOOSVariable("LineEnd", "LINE_END", "", 0);
  AddMOOSVariable("TurnReached", "TURN_REACHED", "", 0);

  // Publish variables
  AddMOOSVariable("TurnPoint", "", "TURN_UPDATE", 0);
  AddMOOSVariable("Stop", "", "FAULT", 0);
  AddMOOSVariable("SurveyPath", "", "SURVEY_UPDATE", 0);
  AddMOOSVariable("StartPath", "", "START_UPDATE", 0);

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SurveyPath::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SurveyPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  RegisterMOOSVariables();
  // Register("FOOBAR", 0);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SurveyPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  UpdateMOOSVariables(NewMail);

//   MOOSMSG_LIST::iterator p;
//   for(p=NewMail.begin(); p!=NewMail.end(); p++) {
//     CMOOSMsg &msg = *p;
//     string key    = msg.GetKey();
//
// #if 0 // Keep these around just for template
//     string comm  = msg.GetCommunity();
//     double dval  = msg.GetDouble();
//     string sval  = msg.GetString();
//     string msrc  = msg.GetSource();
//     double mtime = msg.GetTime();
//     bool   mdbl  = msg.IsDouble();
//     bool   mstr  = msg.IsString();
// #endif
//
//      if(key == "SWATH_WIDTH")
//        cout << "great!";
//
//      else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
//        reportRunWarning("Unhandled Mail: " + key);
//    }



  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SurveyPath::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Process received data
  auto begin_msg = GetMOOSVar("LineBegin");
  if (begin_msg->IsFresh()) {
    #if DEBUG
    MOOSTrace("\n**** Line beginning, starting to record swath ****");
    #endif
    m_recording = true;
  }

  if (m_recording) {
    auto swath_msg = GetMOOSVar("Swath");
    if (swath_msg->IsFresh()) {
      if(InjestSwathMessage(swath_msg->GetStringVal())) {
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
          m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
          m_swath_info["depth"]);
      }
    }
  }

  auto end_msg = GetMOOSVar("LineEnd");
  if (end_msg->IsFresh()) {
    m_recording = false;
    CreateNewPath();
  }

  bool published = PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(published);
}

void SurveyPath::CreateNewPath() {
  m_swath_side = AdvanceSide(m_swath_side);
  #if DEBUG
  MOOSTrace("End of Line, outputting swath points on ---- side.'");
  #endif
  m_swath_record.SaveLast();
  if (m_swath_record.ValidRecord()) {
    // TODO: Check for all swath widths being zero to end area
    // Build full coverage model at some point? Or do this in PathPlan...
    #if DEBUG
    MOOSTrace("Planning next path.\n");
    #endif
    PathPlan planner = PathPlan(m_swath_record, m_swath_side, m_op_region,
      m_swath_overlap, true);
    m_survey_path = planner.GenerateNextPath();
    #if DEBUG
    MOOSTrace("Number of pts in new_path: %d\n",m_survey_path.size());
    #endif
    if (m_survey_path.size() > 0) {
      m_posted_path_str = m_survey_path.get_spec_pts(2);  //2 decimal precision
      SetMOOSVar("SurveyPath", m_posted_path_str, MOOSTime());
      DetermineStartAndTurn();
    } else {
      SetMOOSVar("Stop", "true", MOOSTime());
    }
  }
  m_swath_record.ResetLine();
}

bool SurveyPath::DetermineStartAndTurn() {
  
}

BoatSide SurveyPath::AdvanceSide(BoatSide side) {
  if (side == BoatSide::Stbd) {
   return BoatSide::Port;
  } else if (side == BoatSide::Port) {
   return BoatSide::Stbd;
  }
  return BoatSide::Unknown;
}

bool SurveyPath::InjestSwathMessage(std::string msg) {
  std::vector<std::string> split_msg;
  boost::split(split_msg, msg, boost::is_any_of(";"), boost::token_compress_on);
  if (split_msg.size() != 6) {
    return false;
  }
  for (std::string component : split_msg) {
    std::string param = tolower(biteStringX(component, '='));
    m_swath_info[param] = std::stod(component);
  }
  return true;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool SurveyPath::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "File: pSurveyPath                            \n";
  m_msgs << "============================================ \n";

  m_msgs << m_posted_path_str;

  // ACTable actab(4);
  // actab << "Alpha | Bravo | Charlie | Delta";
  // actab.addHeaderLines();
  // actab << "one" << "two" << "three" << "four";
  // m_msgs << actab.getFormattedString();

  return(true);
}

// std::vector<std::string>& SurveyPath::split(const std::string &s, char delim, std::vector<std::string> &elems) {
//     std::stringstream ss(s);
//     std::string item;
//     while (std::getline(ss, item, delim)) {
//         elems.push_back(item);
//     }
//     return elems;
// }
//
// std::vector<std::string> split(const std::string &s, char delim) {
//     std::vector<std::string> elems;
//     split(s, delim, elems);
//     return elems;
// }
