/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.cpp                                  */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include <iterator>
#include <regex>
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
  m_swath_record(10), m_swath_side{BoatSide::Port}, m_turn_pt_set{false}
{
  m_swath_side = AdvanceSide(m_first_swath_side);
  m_swath_record.SetOutputSide(m_swath_side);
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
    MOOSTrace("pSurveyPath Parameter: " + orig + "\n");
    std::string line  = *p;
    std::string param = toupper(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if(param == "OP_REGION") {
      std::regex pattern("_");
      std::string spaces_added = std::regex_replace(line, pattern, " ");
      MOOSTrace("pSurveyPath After Spaces: " + spaces_added + "\n");
      boost::geometry::read_wkt(spaces_added, m_op_region);
      boost::geometry::correct(m_op_region);
      handled = true;
    }
    else if(param == "FIRST_SIDE") {
      if (toupper(value) == "PORT") {

      }
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
  AddMOOSVariable("NextSwathSide", "", "NEXT_SWATH_SIDE", 0);

  //On Connect to Surver called before this
  registerVariables();
  PostSurveyRegion();

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SurveyPath::OnConnectToServer()
{
  bool published = PublishFreshMOOSVariables();
  return(published);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SurveyPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  RegisterMOOSVariables();
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
    MOOSTrace("**** Line beginning, starting to record swath ****\n");
    #endif
    m_recording = true;
  }

  if (m_recording) {
    auto swath_msg = GetMOOSVar("Swath");
    if (swath_msg->IsFresh()) {
      if(InjestSwathMessage(swath_msg->GetStringVal())) {
        #if DEBUG
        MOOSTrace("pSurveyPath: Recording Swath message\n");
        #endif
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
          m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
          m_swath_info["depth"]);
      }
    }
  } else {
    auto turn_msg = GetMOOSVar("TurnReached");
    if (m_turn_pt_set && turn_msg->IsFresh()) {
      SetMOOSVar("TurnPoint", "point=" + m_turn_pt.get_spec(), MOOSTime());
      m_turn_pt_set = false;
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

void SurveyPath::PostSurveyRegion() {
  m_survey_path.clear();

  #if DEBUG
  std::cout << "Posting Survey Area" << std::endl;
  #endif

  // Survey Region limits (currently only the outer ring)
  auto ext_ring = m_op_region.outer();
  XYSegList survey_limits;
  for (auto poly_vertex : ext_ring) {
    survey_limits.add_vertex(poly_vertex.x(), poly_vertex.y());
  }
  Notify("VIEW_SEGLIST", survey_limits.get_spec_pts(2) + ",label=op_region," +
    "label_color=red,edge_color=red,vertex_color=red,edge_size=2", MOOSTime());

  // Set the first path of the survey
  m_survey_path.add_vertex(survey_limits.get_vx(0), survey_limits.get_vy(0));
  m_survey_path.add_vertex(survey_limits.get_vx(1), survey_limits.get_vy(1));
  SetMOOSVar("SurveyPath", "points=" + m_survey_path.get_spec_pts(2), MOOSTime());

  // Set the alignment lines and turn for the first line
  DetermineStartAndTurn(m_survey_path, true);

  // Set home location = beginning of alignment line
  XYPoint home_pt(m_alignment_line.get_vx(0), m_alignment_line.get_vy(0));
  // Notify("HOME_UPDATE", "station_pt=" + home_pt.get_spec());
  Notify("HOME_UPDATE", "station_pt=" + std::to_string(m_alignment_line.get_vx(0))
    + "," + std::to_string(m_alignment_line.get_vy(0)));

  PostSwathSide();
}

void SurveyPath::CreateNewPath() {
  #if DEBUG
  if (m_swath_record.GetOutputSide() == BoatSide::Stbd) {
    MOOSTrace("End of Line, outputting swath points on stbd side.\n");
  } else if (m_swath_record.GetOutputSide() == BoatSide::Port){
    MOOSTrace("End of Line, outputting swath points on port side.\n");
  }
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
    if (m_survey_path.size() > 2) {
      m_posted_path_str = m_survey_path.get_spec_pts(2);  //2 decimal precision
      SetMOOSVar("SurveyPath", "points=" + m_posted_path_str, MOOSTime());
      DetermineStartAndTurn(m_survey_path);
    } else {
      SetMOOSVar("Stop", "true", MOOSTime());
      #if DEBUG
      MOOSTrace("Path too short, ending survey\n");
      #endif
    }
  }
  m_swath_side = AdvanceSide(m_swath_side);
  PostSwathSide();
  m_swath_record.SetOutputSide(m_swath_side);
  m_swath_record.ResetLine();
}

bool SurveyPath::DetermineStartAndTurn(XYSegList& next_pts, bool post_turn) {
  std::size_t pts_len = next_pts.size();

  // The turn point, extended from the end of the path
  auto end_x = next_pts.get_vx(pts_len-1);
  auto end_y = next_pts.get_vy(pts_len-1);
  EPoint end_heading(end_x - next_pts.get_vx(pts_len-2),
    end_y - next_pts.get_vy(pts_len-2));
  end_heading.normalize();
  end_heading *= m_turn_pt_offset;
  m_turn_pt = XYPoint(end_x + end_heading.x(), end_y + end_heading.y());
  m_turn_pt.set_spec_digits(2);
  m_turn_pt_set = true;
  if (post_turn) {
    SetMOOSVar("TurnPoint", "point=" + m_turn_pt.get_spec(), MOOSTime());
    m_turn_pt_set = false;
  }
  // This is not posted until the current turn point is reached

  // The alignment line, added to the beginning of the path
  EPoint start_heading(next_pts.get_vx(0) - next_pts.get_vx(1),
    next_pts.get_vy(0) - next_pts.get_vy(1));
  start_heading.normalize();
  start_heading *= m_alignment_line_len;
  m_alignment_line.clear();
  m_alignment_line.add_vertex(next_pts.get_vx(0) + start_heading.x(),
    next_pts.get_vy(0) + start_heading.y());
  m_alignment_line.add_vertex(next_pts.get_vx(0), next_pts.get_vy(0));
  SetMOOSVar("StartPath", "points=" + m_alignment_line.get_spec_pts(2), MOOSTime());

  return true;
}

BoatSide SurveyPath::AdvanceSide(BoatSide side) {
  if (side == BoatSide::Stbd) {
   return BoatSide::Port;
  } else if (side == BoatSide::Port) {
   return BoatSide::Stbd;
  }
  return BoatSide::Unknown;
}

void SurveyPath::PostSwathSide() {
  if (m_swath_side == BoatSide::Stbd) {
    SetMOOSVar("NextSwathSide", "stbd", MOOSTime());
  } else if (m_swath_side == BoatSide::Port) {
    SetMOOSVar("NextSwathSide", "port", MOOSTime());
  }
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
