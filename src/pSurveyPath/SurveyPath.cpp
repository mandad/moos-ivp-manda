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
#include "AngleUtils.h"
#include "XYFormatUtilsSegl.h"
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
  m_swath_record(10), m_swath_side{BoatSide::Stbd}, m_turn_pt_set{false},
  m_post_turn_when_ready{false}, m_path_plan_done{false}, m_max_bend_angle{60},
  m_execute_path_plan{false}, m_plan_thread_running{false}
{
  //m_swath_side = AdvanceSide(m_first_swath_side);
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
    double dval  = atof(value.c_str());

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
        m_swath_side = BoatSide::Port;
      } else if (toupper(value) == "STBD" || toupper(value) == "STARBOARD") {
        m_swath_side = BoatSide::Stbd;
      }
      m_swath_record.SetOutputSide(m_swath_side);
      PostSwathSide();
      handled = true;
    }
    else if (param == "FIRST_LINE") {
      if (toupper(value).compare("AUTO") != 0) {
        m_survey_path = string2SegList(value);
        MOOSTrace("First Line Set: " + m_survey_path.get_spec_pts(2) + "\n");
      }
      handled = true;
    }
    else if (param == "OVERLAP_PERCENT" && isNumber(value)) {
      m_swath_overlap = dval/100;
      handled = true;
    }
    else if (param == "PATH_INTERVAL" && isNumber(value)) {
      m_swath_interval = dval;
      m_swath_record = RecordSwath(m_swath_interval);
      handled = true;
    }
    else if (param == "TURN_PT_OFFSET" && isNumber(value)) {
      m_turn_pt_offset = dval;
      handled = true;
    }
    else if (param == "ALIGNMENT_LINE_LEN" && isNumber(value)) {
      m_alignment_line_len = dval;
      handled = true;
    }
    else if (param == "MAX_BEND_ANGLE" && isNumber(value)) {
      m_max_bend_angle = dval;
      handled = true;
    }

    if(!handled && param != "TERM_REPORTING")
      reportUnhandledConfigWarning(orig);
  }

  // Reception variables
  AddMOOSVariable("Swath", "SWATH_WIDTH", "", 0);
  AddMOOSVariable("LineBegin", "LINE_BEGIN", "", 0);
  AddMOOSVariable("LineEnd", "LINE_END", "", 0);
  AddMOOSVariable("TurnReached", "TURN_REACHED", "", 0);
  AddMOOSVariable("AlignmentLineStart", "ALIGN_LINE", "", 0);
  AddMOOSVariable("Heading", "NAV_HEADING", "", 0);
  AddMOOSVariable("DesiredHeading", "DESIRED_HEADING", "", 0);

  // Publish variables
  AddMOOSVariable("TurnPoint", "", "TURN_UPDATE", 0);
  AddMOOSVariable("Stop", "", "FAULT", 0);
  AddMOOSVariable("SurveyPath", "", "SURVEY_UPDATE", 0);
  AddMOOSVariable("StartPath", "", "START_UPDATE", 0);
  AddMOOSVariable("ToStartPath", "", "TO_START_UPDATE", 0);
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
  auto align_msg = GetMOOSVar("AlignmentLineStart");
  auto begin_msg = GetMOOSVar("LineBegin");
  // Need LineBegin for the beginning of the survey, after this it should be
  // triggered by AlignmentLineStart
  if (align_msg->IsFresh() && !m_recording) {
    auto heading_msg = GetMOOSVar("Heading");
    auto desired_msg = GetMOOSVar("DesiredHeading");
    // Don't start logging until actually aligned
    double angle_diff = angleDiff(desired_msg->GetDoubleVal(),
      heading_msg->GetDoubleVal());
    if (angle_diff < 10 || begin_msg->IsFresh()) {
      begin_msg->SetFresh(false);
      align_msg->SetFresh(false);
      if (toupper(align_msg->GetStringVal()) == "TRUE") {
        #if DEBUG
        MOOSTrace("**** Line beginning, starting to record swath ****\n");
        #endif
        m_recording = true;
        m_line_end = false;
      }
    } else {
      #if DEBUG
      MOOSTrace("Turning to line, difference from desired heading: %0.1f\n",
        angle_diff);
      #endif
    }
  }

  if (m_recording) {
    auto swath_msg = GetMOOSVar("Swath");
    if (swath_msg->IsFresh()) {
      swath_msg->SetFresh(false);
      if(InjestSwathMessage(swath_msg->GetStringVal())) {
        #if DEBUG
        //MOOSTrace("pSurveyPath: Recording Swath message\n");
        #endif
        m_swath_record.AddRecord(m_swath_info["stbd"], m_swath_info["port"],
          m_swath_info["x"], m_swath_info["y"], m_swath_info["hdg"],
          m_swath_info["depth"]);
        if (m_line_end && SwathOutsideRegion()) {
          #if DEBUG
          MOOSTrace("**** Ending Recording ****\n");
          #endif
          m_recording = false;
          m_execute_path_plan = true;
        }
      }
    }
  }
  if (m_line_end) {
    auto turn_msg = GetMOOSVar("TurnReached");
    if (turn_msg->IsFresh()) {
      turn_msg->SetFresh(false);
      if (m_path_plan_done) {
        PostTurnPoint();
      } else {
        //Hold until processing is done
        MOOSTrace("pSurveyPath: Holding until processing complete\n");
        SetMOOSVar("Stop", "true", MOOSTime());
        m_post_turn_when_ready = true;

        // If we still don't have the swath outside the region, plan anyway
        #if DEBUG
        MOOSTrace("**** Ending Recording ****\n");
        #endif
        m_recording = false;
        m_execute_path_plan = true;
      }
    } else if (m_post_turn_when_ready && m_path_plan_done) {
      PostTurnPoint();
    }
  }

  auto end_msg = GetMOOSVar("LineEnd");
  if (end_msg->IsFresh()) {
    end_msg->SetFresh(false);
    m_line_end = true;
    m_path_plan_done = false;
  }

  if (m_execute_path_plan) {
    m_execute_path_plan = false;
    //clear the last raw path
    if (m_raw_survey_path.size() > 0) {
      Notify("VIEW_SEGLIST", m_raw_survey_path.get_spec_pts(2)
          + "active=false,label=raw_path,", MOOSTime());
    }
    #if DEBUG
    MOOSTrace("pSurveyPath: Launching Path Processing Thread\n");
    #endif
    //I 'm sure there must be something non threadsafe about this, I haven't
    // thought it through much
    m_plan_thread_running = true;
    /*m_path_plan_thread = */std::thread(&SurveyPath::CreateNewPath, this).detach();
    #if DEBUG
    MOOSTrace("pSurveyPath: Thread Launched\n");
    #endif
    //m_path_plan_thread.join();
  }

  bool published = PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(published);
}

bool SurveyPath::SwathOutsideRegion() {
  std::pair<XYPoint, XYPoint> swath_edges = m_swath_record.LastOuterPoints();
  BPoint port_edge(swath_edges.first.x(), swath_edges.first.y());
  BPoint stbd_edge(swath_edges.second.x(), swath_edges.second.y());

  auto outer_ring = m_op_region.outer();
  bool outside_region = !boost::geometry::within(port_edge, outer_ring);
  outside_region = outside_region && !boost::geometry::within(stbd_edge, outer_ring);

  return outside_region;
}

void SurveyPath::PostSurveyRegion() {
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
  if (m_survey_path.size() < 2) {
    m_survey_path.clear();
    m_survey_path.add_vertex(survey_limits.get_vx(0), survey_limits.get_vy(0));
    m_survey_path.add_vertex(survey_limits.get_vx(1), survey_limits.get_vy(1));
  }
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

void SurveyPath::PostTurnPoint() {
  #if DEBUG
  MOOSTrace("pSurveyPath: Posting Turn Point\n");
  #endif
  SetMOOSVar("TurnPoint", "point=" + m_turn_pt.get_spec(), MOOSTime());
  // Don't restart the survey if there are no points
  if (m_survey_path.size() > 2) {
    SetMOOSVar("Stop", "false", MOOSTime());
  }
  if (m_raw_survey_path.size() > 0) {
    Notify("VIEW_SEGLIST", m_raw_survey_path.get_spec_pts(2) + ",label=raw_path," +
      "label_color=darkgoldenrod,edge_color=darkgoldenrod,vertex_color=yellow,edge_size=2," +
      "vertex_size=3", MOOSTime());
  }
  m_post_turn_when_ready = false;
  m_path_plan_done = false;
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
      m_swath_overlap, m_max_bend_angle, true);
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
    m_swath_side = AdvanceSide(m_swath_side);
    PostSwathSide();
    m_swath_record.SetOutputSide(m_swath_side);
    m_swath_record.ResetLine();
    m_raw_survey_path = planner.GetRawPath();
  }
  m_plan_thread_running = false;
  m_path_plan_done = true;
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

  XYSegList to_start_path;
  to_start_path.add_vertex(m_alignment_line.get_vx(0), m_alignment_line.get_vy(0));
  SetMOOSVar("ToStartPath", "points=" + to_start_path.get_spec_pts(2), MOOSTime());

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

  if (!m_plan_thread_running) {
    m_msgs << "Last Path Length: " << m_survey_path.size() << "\n\n";
  }

  if (m_recording) {
    m_msgs << "Recording Swath Info\n";
  }

  // ACTable actab(4);
  // actab << "Alpha | Bravo | Charlie | Delta";
  // actab.addHeaderLines();
  // actab << "one" << "two" << "three" << "four";
  // m_msgs << actab.getFormattedString();

  return(true);
}
