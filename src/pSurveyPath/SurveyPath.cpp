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

SurveyPath::SurveyPath() : m_first_swath_side{BoatSide::Stbd},
  m_swath_interval{10}, m_alignment_line_len{10}, m_turn_pt_offset{15},
  m_remove_in_coverage{false}, m_swath_overlap{0.2}, m_line_end{false},
  m_line_begin{false}, m_turn_reached{false}, m_recording{false},
  m_swath_record(10)
{
  m_next_swath_side = AdvanceSide(m_first_swath_side);
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
  AddMOOSVariable("TurnUpdate", "", "TURN_UPDATE", 0);
  AddMOOSVariable("Stop", "", "FAULT", 0);
  AddMOOSVariable("SurveyUpdate", "", "SURVEY_UPDATE", 0);
  AddMOOSVariable("StartUpdate", "", "START_UPDATE", 0);

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

  auto swath_msg = GetMOOSVar("Swath");
  if (swath_msg->IsFresh()) {
    InjestSwathMessage(swath_msg->GetStringVal());
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SurveyPath::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool published = PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(published);
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
  return true;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool SurveyPath::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "File:                                        \n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

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
