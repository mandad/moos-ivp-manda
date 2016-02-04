/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SpeedControl.cpp                                */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "SpeedControl.h"
#include "AngleUtils.h"
#include <cmath>

#define ANGLE_BINS 20
#define HISTORY_TIME 60
#define AVERAGING_LEN 3
#define MAX_FLAT_SLOPE 0.8 //m/s^2

SpeedControl::SpeedControl() : m_thrust_output(0),  m_first_run(true), 
                               m_thrust_map_set(true), m_max_thrust(100), 
                               has_adjust(false) {
  //  = 0;
  // m_first_run = true;
  // m_thrust_map_set = false;
}

double SpeedControl::Run(double desired_speed, double speed, double heading,
                         double current_time, bool turning) {
  double thrust = 0;
  bool speed_is_level = false;

  if (m_first_run && m_thrust_map_set) {
    InitControls(speed, heading);
    m_first_run = false;
    m_time_at_speed = 0;

    thrust = m_thrust_map.getThrustValue(desired_speed);
  } else if (m_previous_desired_speed == desired_speed) {
    m_time_at_speed = current_time - m_speed_hist.back().m_time;
  } else {
    //need to do something for small speed changes (use same offset)
    m_time_at_speed = 0;
    m_speed_hist.clear();
    has_adjust = false;
    //Should add heading history value here
    thrust = m_thrust_map.getThrustValue(desired_speed);
  }

  //need to figure out way to prevent wind up
  m_speed_hist.emplace_front(desired_speed, speed, heading, current_time);
  while (current_time - m_speed_hist.back().m_time > HISTORY_TIME) {
    m_speed_hist.pop_back();
  }

  double speed_avg(0);
  double speed_slope(0);
  bool history_valid = SpeedHistInfo(AVERAGING_LEN, speed_slope, speed_avg);
  double time_at_heading = 0;
  //Largest from calm day straight = 0.04
  //Largest from rough day straight = 0.10
  if (!turning && history_valid && fabs(speed_slope) < MAX_FLAT_SLOPE) {
    speed_is_level = true;
    time_at_heading = TimeAtHeading(10);
  }

  //round to nearest ANGLE_BINS and add to history
  //only do this if past a certain time
  if (speed_is_level && time_at_heading > AVERAGING_LEN) {
    int binned_direction = int(std::round(angle360(heading) / ANGLE_BINS));
    m_direction_average[binned_direction] = std::make_pair(
      m_direction_average[binned_direction].first + speed, 
      m_direction_average[binned_direction].second + 1);
  }

  //Update before the next cycle
  m_previous_desired_speed = desired_speed;

  // Set the output speed (if zero, turn thrust off, no need for control)
  // This may need to be changed if we want to hold against currents
  if (desired_speed == 0) {
    m_thrust_output = 0;
  } else {
    m_thrust_output = thrust;
  }
  MOOSAbsLimit(m_thrust_output, m_max_thrust);
  return m_thrust_output;
}

void SpeedControl::InitControls(double speed, double heading) {
  for (int direction = 0; direction < int(std::round(360 / ANGLE_BINS)); 
       direction++) {
    m_direction_average[direction] = std::make_pair(0,0);
  }

}


void SpeedControl::SetParameters(std::string thrust_map, double max_thrust) {
  if (thrust_map != "") {
    bool map_ok = m_thrust_map.injestMapString(thrust_map);
    if (!map_ok) {
      MOOSTrace("Speed Control: Error in Thrust Map");
      m_thrust_map_set = true;
    }
  }

  m_max_thrust = max_thrust;
}

bool SpeedControl::SpeedHistInfo(double time_range, double &slope, 
                                 double &average) {
  if (time_range <= 0) {
    return false;
  }

  double latest_time = m_speed_hist.front().m_time;
  double latest_speed = m_speed_hist.front().m_speed;
  double past_speed = 0;
  double past_time = latest_time - time_range;
  double speed_sum = 0;
  double num_records = 0;
  bool valid_history = false;

  std::list<SpeedRecord>::iterator record;
  for(record = m_speed_hist.begin(); record != m_speed_hist.end(); record++) {
    speed_sum += record->m_speed;
    num_records++;
    if ((latest_time - record->m_time) > time_range) {
      past_speed = record->m_speed;
      past_time = record->m_time;
      valid_history = true;
      break;
    }
  }
  
  if (valid_history) {
    slope = (latest_speed - past_speed) / (latest_time - past_time);
    average = speed_sum / num_records;
  }

  return valid_history;
}

double SpeedControl::TimeAtHeading(double allowable_range) {
  double current_heading = m_speed_hist.front().m_heading;
  double oldest_time = m_speed_hist.front().m_time;

  std::list<SpeedRecord>::iterator record;
  for(record = m_speed_hist.begin(); record != m_speed_hist.end(); record++) {
    oldest_time = record->m_time;
    if (fabs(record->m_heading - current_heading) > allowable_range) {
      break;
    }
  }

  return m_speed_hist.front().m_time - oldest_time;
}

// double SpeedControl::