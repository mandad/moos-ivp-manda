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
#define SPEED_TOLERANCE 0.05

SpeedControl::SpeedControl() : m_thrust_output(0),  m_first_run(true), 
                               m_thrust_map_set(true), m_max_thrust(100), 
                               m_has_adjust(false), m_initial_speed(0) {
  InitControls();
}

double SpeedControl::Run(double desired_speed, double speed, double heading,
                         double current_time, bool turning) {
  //MOOSTrace("Speed Control: Desired Speed = %.2f\n", desired_speed);
  if (isnan(desired_speed)) {
    MOOSTrace("Speed Control: Passed NaN Desired Speed\n");
    return m_thrust_output;
  }
  //default is to not change the thrust output
  double thrust = m_thrust_output;
  bool speed_is_level = false;
  int binned_direction = int(std::round(angle360(heading) / ANGLE_BINS));
  if (binned_direction > m_direction_average.size())
    binned_direction = 0;
  // MOOSTrace("Speed Control: Binned Direction = %i\n", binned_direction);

  //need to figure out way to prevent wind up when under human control
  m_speed_hist.emplace_front(desired_speed, speed, heading, current_time);
  while (current_time - m_speed_hist.back().m_time > HISTORY_TIME) {
    m_speed_hist.pop_back();
  }

  double speed_avg = 0;
  double speed_slope = 0;
  bool history_valid = SpeedHistInfo(AVERAGING_LEN, speed_slope, speed_avg);
  double time_at_heading = TimeAtHeading(10);
  MOOSTrace("Speed Control: Time At Heading = %.2f\n", time_at_heading);
  //Largest from calm day straight = 0.04
  //Largest from rough day straight = 0.10
  if (!turning && history_valid && fabs(speed_slope) < MAX_FLAT_SLOPE) {
    speed_is_level = true;
  }
  // If we have more than a 1 sec diff in time at the heading (i.e. turning)
  if (turning || (m_prev_time_at_heading - time_at_heading) > 1) {
    m_turn_began = true;
  }
  // if (turning) {
  //   m_has_adjust = false;
  //   m_turning_time = current_time
  // }


  if (m_first_run && m_thrust_map_set) {
    
    m_first_run = false;
    m_turn_began = false;
    // m_time_at_speed = 0;

    m_initial_speed = desired_speed;
    thrust = m_thrust_map.getThrustValue(m_initial_speed);
    m_thrust_change_time = current_time;
    MOOSTrace("Speed Control: First Run, Desired Speed = %.2f Thrust = %.2f\n", 
      desired_speed, thrust);
  } else if (m_previous_desired_speed != desired_speed || 
             (time_at_heading > AVERAGING_LEN && m_turn_began && !turning)) {
    // m_time_at_speed = current_time - m_speed_hist.back().m_time;
    //We have changed desired speeds
    //need to do something for small speed changes (use same offset)
    // m_time_at_speed = 0;
    // m_speed_hist.clear();
    m_has_adjust = false;
    m_turn_began = false;
    //Should add heading history value here
    double speed_diff_avg = 0;
    if (m_direction_average[binned_direction].second > 0)
      speed_diff_avg = m_direction_average[binned_direction].first /
        m_direction_average[binned_direction].second;
    m_initial_speed = desired_speed - speed_diff_avg;
    thrust = m_thrust_map.getThrustValue(m_initial_speed);
    MOOSTrace("Speed Control: New Setting, Inital Speed = %.2f Thrust = %.2f Speed Diff Avg = %.2f Binned Dir = %i\n", 
      m_initial_speed, thrust, speed_diff_avg, binned_direction);
  } else if (speed_is_level && time_at_heading > (2 * AVERAGING_LEN)
      && (current_time - m_thrust_change_time) > (2 * AVERAGING_LEN)) {
    //round to nearest ANGLE_BINS and add to history
    //only do this if past a certain time
    double map_speed_diff = speed - m_thrust_map.getSpeedValue(m_thrust_output);
    //this will overflow after a long time
    m_direction_average[binned_direction] = std::make_pair(
      m_direction_average[binned_direction].first + map_speed_diff, 
      m_direction_average[binned_direction].second + 1);

    
    if (!m_has_adjust) {
      double des_speed_diff = desired_speed - speed_avg;
      // TODO: Maybe don't do this when within SPEED_TOLERANCE?
      // Probably should also take into account new heading offsets
      thrust = m_thrust_map.getThrustValue(m_initial_speed + des_speed_diff);
      m_has_adjust = true;
      MOOSTrace("Speed Control: First Adjust, Speed Diff = %.2f Thrust = %.2f\n", 
        des_speed_diff, thrust);
    } else if ((current_time - m_thrust_change_time) > (3 * AVERAGING_LEN)) {
      //Do minor adjustments after the first big one
      //Rewrite the speed_slope & avg here, maybe should be new vars?
      if (time_at_heading > (AVERAGING_LEN * 3)) {
        history_valid = SpeedHistInfo(AVERAGING_LEN * 2, speed_slope, speed_avg);
        if (history_valid) {
          //Note that this has a longer averaging period then the first adjust
          double des_speed_diff_long = desired_speed - speed_avg;
          MOOSTrace("Speed Control: Small Adjust, Speed Diff = %.2f", des_speed_diff_long);
          double delta_t = current_time - m_previous_time;
          if (fabs(des_speed_diff_long) > SPEED_TOLERANCE) {
            double mapped_speed = m_thrust_map.getSpeedValue(m_thrust_output);
            thrust = m_thrust_map.getThrustValue(mapped_speed + des_speed_diff_long);
            // double thrust_slope = m_thrust_map.getSlopeAtThrust(m_thrust_output);
            //Adjust based on the differential
            // thrust = m_thrust_output + des_speed_diff_long * thrust_slope;
            MOOSTrace(" Thrust = %.2f", thrust);
          }
          MOOSTrace("\n");
        }
      }
    }
  }

  //Update before the next cycle
  m_previous_desired_speed = desired_speed;
  m_previous_time = current_time;
  m_prev_time_at_heading = time_at_heading;

  // Set the output speed (if zero, turn thrust off, no need for control)
  // This may need to be changed if we want to hold against currents
  MOOSAbsLimit(thrust, m_max_thrust);
  if (thrust != m_thrust_output) {
      m_thrust_change_time = current_time;
  }
  if (desired_speed == 0) {
    m_thrust_output = 0;
  } else {
    m_thrust_output = thrust;
  }
  return m_thrust_output;
}

void SpeedControl::InitControls() {
  for (int direction = 0; direction < int(std::round(360 / ANGLE_BINS)); 
       direction++) {
    MOOSTrace("Speed Control: Setting Direction = %i\n", direction);
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

std::string SpeedControl::AppCastMessage() {
  std::stringstream message;
  message << "Speed Control Enabled\n";
  message << "\nSpeed Averages:\n";

  std::map<int, std::pair<double, int>>::iterator dir_avgs;
  for (dir_avgs = m_direction_average.begin(); 
    dir_avgs != m_direction_average.end(); dir_avgs++) {
    double avg_offset = 0;
    if (dir_avgs->second.second != 0) {
      avg_offset = dir_avgs->second.first / dir_avgs->second.second;
    }
    message << dir_avgs->first * ANGLE_BINS << ": " << avg_offset << std::endl;
  }

  return message.str();
}