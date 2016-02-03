/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SpeedControl.cpp                                */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "SpeedControl.h"
#include "AngleUtils.h"

#define ANGLE_BINS 20

SpeedControl::SpeedControl() : m_thrust_output(0),  m_first_run(true), 
                               m_thrust_map_set(true) {
  //  = 0;
  // m_first_run = true;
  // m_thrust_map_set = false;
}

double SpeedControl::Run(double desired_speed, double speed, double heading,
                         double current_time) {
  double thrust = 0;

  if (m_first_run && m_thrust_map_set) {
    InitControls(speed, heading);
    m_first_run = false;
    m_time_at_speed = 0;

    thrust = m_thrust_map.getThrustValue(desired_speed);
  } else {
    m_time_at_speed = current_time - m_speed_hist.back().m_time;
  }

  //need to figure out way to prevent wind up
  m_speed_hist.emplace_front(desired_speed, speed, heading, current_time);
  // m_des_speed_hist.push_front(desired_speed);
  // m_time_hist.push_front(current_time);

  //round to nearest ANGLE_BINS and add to history
  //only do this if past a certain time
  int binned_direction = angle360(int(((heading + ANGLE_BINS/2) / ANGLE_BINS) 
    * ANGLE_BINS));
  m_direction_average[binned_direction] = std::make_pair(
    m_direction_average[binned_direction].first + speed, 
    m_direction_average[binned_direction].second + 1);

  m_thrust_output = thrust;
  return m_thrust_output;
}

void SpeedControl::InitControls(double speed, double heading) {
  //this could also just use 360/ANGLE_BINS as indicies
  for (int direction = 0; direction < 360; direction += ANGLE_BINS) {
    m_direction_average[direction] = std::make_pair(0,0);
  }


}


void SpeedControl::SetParameters(std::string thrust_map) {
  bool map_ok = m_thrust_map.injestMapString(thrust_map);
  if (!map_ok) {
    MOOSTrace("Speed Control: Error in Thrust Map");
    m_thrust_map_set = true;
  }
}