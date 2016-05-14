/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH/NOAA                                             */
/*    FILE: SonarFilter.cpp                                        */
/*    DATE: 2015-08-26                                     */
/************************************************************/

#include <iterator>
#include <numeric>
#include <algorithm>
#include <math.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "SonarFilter.h"
// #include <boost/foreach.hpp>
// #include <boost/accumulators/accumulators.hpp>
// #include <boost/accumulators/statistics/variance.hpp>

using namespace std;
// Used to reject noise, expecially in single beam systems
#define MIN_DEPTH_LIMIT 0.5
#define DEBUG false
// using namespace boost::accumulators;

//---------------------------------------------------------
// Constructor

SonarFilter::SonarFilter()
{
  m_fresh_depth = false;
  m_cycles_since_last = 0;
  m_std_limit = 2;
  m_filter_len = 10;
  m_sim_swath_angle = 70 * M_PI / 180;
  m_last_msg[0] = '\0';
  m_last_depth = -1;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SonarFilter::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  UpdateMOOSVariables(NewMail);

  /*
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
   */

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SonarFilter::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SonarFilter::Iterate()
{
  AppCastingMOOSApp::Iterate();

  CMOOSVariable * pDepth = GetMOOSVar("Depth");
  double dfDepth = -1;
  if (pDepth->IsFresh()) {
    dfDepth = pDepth->GetDoubleVal();
    pDepth->SetFresh(false);
    // This sets a static 0.5m min depth at this time
    if (dfDepth > MIN_DEPTH_LIMIT) {
      m_last_depth = dfDepth;
      m_fresh_depth = m_nadir_filter.IngestValue(dfDepth);
    }
  }
  pDepth = GetMOOSVar("Depth_Port");
  dfDepth = -1;
  if (pDepth->IsFresh()) {
    dfDepth = pDepth->GetDoubleVal();
    pDepth->SetFresh(false);
    if (dfDepth > MIN_DEPTH_LIMIT) {
      m_fresh_depth = m_port_filter.IngestValue(dfDepth);
    }
  }
  pDepth = GetMOOSVar("Depth_Stbd");
  dfDepth = -1;
  if (pDepth->IsFresh()) {
    dfDepth = pDepth->GetDoubleVal();
    pDepth->SetFresh(false);
    if (dfDepth > MIN_DEPTH_LIMIT) {
      m_fresh_depth = m_stbd_filter.IngestValue(dfDepth) || m_fresh_depth;
    }
  }

  if (m_fresh_depth) {
    string swath_message = GenerateSwathMessage();
    SetMOOSVar("Swath", swath_message, MOOSTime());
    m_fresh_depth = false;
  }

  bool published = PublishFreshMOOSVariables();

  AppCastingMOOSApp::PostReport();
  return(published);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SonarFilter::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "FILTERLEN") {
      m_filter_len = atoi(value.c_str());
      handled = true;
    }
    else if(param == "STDEVLIMIT") {
      m_std_limit = atof(value.c_str());
      handled = true;
    } else if (param == "SIMSWATHANGLE") {
      // Convert the angle to radians
      m_sim_swath_angle = atof(value.c_str()) * M_PI / 180;
      handled = true;
    } else if (param == "SONARTYPE") {

    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  AddMOOSVariable("X", "NAV_X", "", 0);
  AddMOOSVariable("Y", "NAV_Y", "", 0);
  AddMOOSVariable("Heading", "NAV_HEADING", "", 0);
  AddMOOSVariable("Depth", "SONAR_DEPTH_M", "", 0);
  AddMOOSVariable("Depth_Stbd", "SONAR_DEPTH_STBD_M", "", 0);
  AddMOOSVariable("Depth_Port", "SONAR_DEPTH_PORT_M", "", 0);
  AddMOOSVariable("SonarWidth", "SONAR_WIDTH", "", 0);
  AddMOOSVariable("Swath", "", "SWATH_WIDTH", 0);

  m_nadir_filter = StDevFilter(m_filter_len, m_std_limit, 1/GetAppFreq());
  m_port_filter = StDevFilter(m_filter_len, m_std_limit, 1/GetAppFreq());
  m_stbd_filter = StDevFilter(m_filter_len, m_std_limit, 1/GetAppFreq());

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SonarFilter::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  RegisterMOOSVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SonarFilter::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "Sonar Filter                                 \n";
  m_msgs << "============================================ \n";

  // ACTable actab(4);
  // actab << "Alpha | Bravo | Charlie | Delta";
  // actab.addHeaderLines();
  // actab << "one" << "two" << "three" << "four";
  // m_msgs << actab.getFormattedString();

  m_msgs << "Last Nadir Depth: " << m_last_depth << "\n";
  m_msgs << "Nadir Filtered: " << m_nadir_filter.FilteredValue() << "\n";
  m_msgs << "Port Filtered: " << m_port_filter.FilteredValue() << "\n";
  m_msgs << "Stbd Filtered: " << m_stbd_filter.FilteredValue() << "\n";
  m_msgs << "\nOutput String: \n" << m_last_msg << "\n";

  return(true);
}

void SonarFilter::InjestDepthVal(double depth) {
  // Reject depths that are rejected by the sonar (0 depth)
  MOOSTrace("SonarFilt - Injesting Depth\n");
  if (depth > 0) {
    double std = GetStDev(&m_all_depths);
    // Need to add values before we can compute stdev
    if (m_all_depths.size() >= 2) {
      MOOSTrace("SonarFilt - Testing StDev\n");
      // Only call a depth good if it is within the stdev limit
      //TODO: Maybe add a fixed factor since the stdev could be low (zero)
      if (depth <= (m_last_valid_depth + std * m_std_limit) &&
        depth >= (m_last_valid_depth - std * m_std_limit)) {
        MOOSTrace("SonarFilt - Have Valid Depth: %0.2f\n", depth);
        m_fresh_depth = true;
        m_cycles_since_last = 0;
        m_last_valid_depth = depth;
      } else {
        MOOSTrace("SonarFilt - Throwing Out Invalid Depth: %0.2f \n", depth);
        if (++m_cycles_since_last > 10) {
          m_last_valid_depth = GetMean(&m_all_depths);
          m_cycles_since_last = 0;
        }
      }
    } else {
      m_last_valid_depth = depth;
    }
    //Add it anyway in case this is a trend
    m_all_depths.push_front(depth);
    if (m_all_depths.size() > m_filter_len) {
      m_all_depths.pop_back();
    }
  }
}

double SonarFilter::GetStDev(list<double> * v) {
  if (v->size() < 2) {
    // Limit for first few variables
    return -1;
  }

  double mean = GetMean(v);

  double sq_sum = std::inner_product(v->begin(), v->end(), v->begin(), 0.0);
  double stdev = std::sqrt(sq_sum / v->size() - mean * mean);

  // Supposedly better:
  // std::vector<double> diff(v.size());
  // std::transform(v.begin(), v.end(), diff.begin(),
  //                std::bind2nd(std::minus<double>(), mean));
  // double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  // double stdev = std::sqrt(sq_sum / v.size());

  return stdev;
}

double SonarFilter::GetMean(list<double> * v) {
  if (v->size() < 2) {
    return v->front();
  }
  double sum = std::accumulate(v->begin(), v->end(), 0.0);
  double mean = sum / v->size();
  return mean;
}

string SonarFilter::GenerateSwathMessage() {
  char message [200];

  CMOOSVariable * x_var = GetMOOSVar("X");
  CMOOSVariable * y_var = GetMOOSVar("Y");
  CMOOSVariable * heading_var = GetMOOSVar("Heading");
  //CMOOSVariable * depth_var = GetMOOSVar("Depth");

  // There is a danger that the variables could not be set yet
  double swath_width_port = 0;
  double swath_width_stbd = 0;
  // Assume if one side is set, then both are
  if (m_port_filter.FilteredValue() < 0) {
    swath_width_port = tan(m_sim_swath_angle) * m_nadir_filter.FilteredValue();
    swath_width_stbd = swath_width_port;
  } else {
    swath_width_port = tan(m_sim_swath_angle) * m_port_filter.FilteredValue();
    swath_width_stbd = tan(m_sim_swath_angle) * m_stbd_filter.FilteredValue();
  }

  double post_depth = m_nadir_filter.FilteredValue();
  if (post_depth < 0) {
    post_depth = (m_port_filter.FilteredValue() + m_stbd_filter.FilteredValue()) * 0.5;
  }

  snprintf (message, 200, "x=%0.2f;y=%0.2f;hdg=%0.2f;port=%0.1f;stbd=%0.1f;depth=%0.2f",
    x_var->GetDoubleVal(), y_var->GetDoubleVal(), heading_var->GetDoubleVal(),
    swath_width_port, swath_width_stbd, post_depth);

  strncpy(m_last_msg, message, sizeof(message));
  string msg_string(message);
  #if DEBUG
  MOOSTrace("SonarFilt - GeneratedMessage:\n");
  MOOSTrace(msg_string + "\n");
  #endif
  return msg_string;
}
