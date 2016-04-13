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
#define MAX_FLAT_SLOPE 0.08 //m/s^2
#define SPEED_TOLERANCE 0.05
#define HEADING_TOLERANCE 7
#define DEBUG true

SpeedControl::SpeedControl() : m_thrust_output(0),  m_first_run(true),
                               m_thrust_map_set(true), m_max_thrust(100),
                               m_initial_speed(0), m_turn_began(false),
                               m_turn_finished(false), m_adjustment_state(0),
                               m_use_thrust_map_only(false), m_previous_desired_speed(0),
                               m_previous_desired_heading(0),
                               m_current_estimate(ANGLE_BINS, 3600) {
  InitControls();
}

double SpeedControl::Run(double desired_speed, double speed, double desired_heading,
                         double heading, double current_time, bool turning,
                         double course_over_ground) {
  if (DEBUG)
    MOOSTrace("Speed Control: Desired Speed = %.2f\n", desired_speed);
  if (isnan(desired_speed) || !m_thrust_map_set) {
    if (DEBUG)
      MOOSTrace("Speed Control: Passed NaN Desired Speed or No Thrust Map\n");
    return m_thrust_output;
  }

  if (DEBUG)
    MOOSTrace("Speed Control: Desired Heading = %.2f\n", desired_heading);
  heading = angle360(heading);
  desired_heading = angle360(desired_heading);
  //default is to not change the thrust output
  double thrust = m_thrust_output;
  bool speed_is_level = false;
  bool heading_is_steady = false;

  //need to figure out way to prevent wind up when under human control
  m_speed_hist.emplace_front(desired_speed, speed, heading, current_time);
  while (m_speed_hist.size() > 1 && current_time - m_speed_hist.back().m_time > HISTORY_TIME) {
    m_speed_hist.pop_back();
  }
  if (DEBUG)
    MOOSTrace("Speed Control: Speed History Size %d\n", m_speed_hist.size());

  double speed_avg = 0;
  double speed_slope = 0;
  bool history_valid = SpeedHistInfo(2 * AVERAGING_LEN, speed_slope, speed_avg);
  //Largest from calm day straight = 0.04
  //Largest from rough day straight = 0.10
  if (history_valid && fabs(speed_slope) < MAX_FLAT_SLOPE) {
    speed_is_level = true;
    if (DEBUG)
      MOOSTrace("Speed Steady\n");
  }

  if (DEBUG)
    MOOSTrace("Speed Control: Debug 1\n");

  double time_at_heading = TimeAtHeading(HEADING_TOLERANCE);
  double time_since_thrust_change = current_time - m_thrust_change_time;

  if (DEBUG)
    MOOSTrace("Speed Control: Debug 2\n");

  // If we have more than a 1 sec diff in time at the heading (i.e. turning)
  // Maybe use 2x avg len for time?
  //|| (m_prev_time_at_heading - time_at_heading) > 1
  // TODO: account for desired_heading = 0 when stopped
  if (time_at_heading > AVERAGING_LEN
      && HeadingAbsDiff(desired_heading, heading) < HEADING_TOLERANCE) {
    heading_is_steady = true;
    if (DEBUG)
      MOOSTrace("Heading Steady\n");
  }

  if (DEBUG)
    MOOSTrace("Speed Control: Debug 3, previous_des: %0.2f\n", m_previous_desired_speed);

  // Determine state
  // 0 = First run, desired speed adjusted, desired heading adjusted
  // 1 = Initial setpoint, waiting for first adjust
  // 2 = Time to do first adjust
  // 3 = Freely do minor adjustments when necessary
  if (fabs(m_previous_desired_speed - desired_speed) > SPEED_TOLERANCE
      || HeadingAbsDiff(desired_heading, m_previous_desired_heading)
      > HEADING_TOLERANCE) {
    m_adjustment_state = 0;
    if (DEBUG)
      MOOSTrace("Speed Control: Debug 3.1\n");
  } else if (heading_is_steady && speed_is_level && m_adjustment_state == 1) {
    m_adjustment_state = 2;
    if (DEBUG)
      MOOSTrace("Speed Control: Debug 3.2\n");
  }
  if (DEBUG)
    MOOSTrace("Speed Control: Debug 3.3\n");
  if (DEBUG)
    MOOSTrace("Speed Control: Time At Heading = %.2f State: %i\n",
      time_at_heading, m_adjustment_state);

  if (m_adjustment_state > 1 && speed_is_level &&
      time_since_thrust_change > 2 * AVERAGING_LEN) {
    // Add this value to history for current estimation
    double speed_est = m_thrust_map.getSpeedValue(m_thrust_output);
    SpeedInfoRecord hist_record(current_time, speed, speed_est, heading,
      course_over_ground);
    m_current_estimate.SaveHistory(hist_record);
  }

  if (DEBUG)
    MOOSTrace("Speed Control: Debug 4\n");

  if (m_adjustment_state == 0) {
    //We have changed desired speeds
    //TODO: need to do something for small speed changes (use same offset)
    m_turn_began = false;
    m_turn_finished = false;
    //Add heading history value for offset
    double speed_diff_avg = m_current_estimate.GetSpeedDiff(desired_heading);
    m_initial_speed = desired_speed - speed_diff_avg;
    thrust = m_thrust_map.getThrustValue(m_initial_speed);
    if (DEBUG)
      MOOSTrace("Speed Control: New Setting, Inital Speed = %.2f Thrust = %.2f Speed Diff Avg = %.2f\n",
        m_initial_speed, thrust, speed_diff_avg);
    m_adjustment_state = 1;
  } else if (m_adjustment_state == 2 && time_at_heading > (2 * AVERAGING_LEN)
      && time_since_thrust_change > (2 * AVERAGING_LEN)) {

    history_valid = SpeedHistInfo(AVERAGING_LEN, speed_slope, speed_avg);
    double des_speed_diff = desired_speed - speed_avg;

    // Probably should also take into account new heading offsets
    if (fabs(des_speed_diff) > SPEED_TOLERANCE) {
      thrust = m_thrust_map.getThrustValue(m_initial_speed + des_speed_diff);
      if (DEBUG)
        MOOSTrace("Speed Control: First Adjust, Speed Diff = %.2f Thrust = %.2f\n",
          des_speed_diff, thrust);
    }
    // Pretend we change so that the small adjustment has to wait
    m_thrust_change_time = current_time;
    m_adjustment_state = 3;
  } else if (m_adjustment_state == 3 && heading_is_steady && speed_is_level
            && time_since_thrust_change > (3 * AVERAGING_LEN)) {
    // Do minor adjustments after the first big one
    // Assume the change doesn't take more than AVERAGING_LEN to manifest since
    // it is small
    //Rewrite the speed_slope & avg here, maybe should be new vars?
    //history_valid = SpeedHistInfo(AVERAGING_LEN * 2, speed_slope, speed_avg);
    if (history_valid) {
      //Note that this has a longer averaging period then the first adjust
      double des_speed_diff_long = desired_speed - speed_avg;
      if (DEBUG)
        MOOSTrace("Speed Control: Small Adjust, Speed Diff = %.2f", des_speed_diff_long);
      double delta_t = current_time - m_previous_time;
      if (fabs(des_speed_diff_long) > SPEED_TOLERANCE) {
        double mapped_speed = m_thrust_map.getSpeedValue(m_thrust_output);
        thrust = m_thrust_map.getThrustValue(mapped_speed + des_speed_diff_long);
        // double thrust_slope = m_thrust_map.getSlopeAtThrust(m_thrust_output);
        //Adjust based on the differential
        // thrust = m_thrust_output + des_speed_diff_long * thrust_slope;
        if (DEBUG)
          MOOSTrace(" Thrust = %.2f", thrust);
      }
      if (DEBUG)
        MOOSTrace("\n");
    }
  }

  //Update before the next cycle
  m_previous_desired_speed = desired_speed;
  m_previous_time = current_time;
  m_prev_time_at_heading = time_at_heading;
  m_previous_desired_heading = desired_heading;

  // Set this here to still allow current estimation, or possibly on the fly
  // switching.  Overrides the settings above.
  if (m_use_thrust_map_only) {
    thrust = m_thrust_map.getThrustValue(desired_speed);
  }

  MOOSAbsLimit(thrust, m_max_thrust);
  if (thrust != m_thrust_output) {
      m_thrust_change_time = current_time;
  }

  if (DEBUG)
    MOOSTrace("Speed Control: Done Run Call\n");

  //Experiment to allow it to compete with currents
  m_thrust_output = thrust;
  return m_thrust_output;
}

void SpeedControl::InitControls() {
  // for (int direction = 0; direction < int(std::round(360 / ANGLE_BINS));
  //      direction++) {
  //   MOOSTrace("Speed Control: Setting Direction = %i\n", direction);
  //   m_direction_average[direction] = std::make_pair(0,0);
  // }

}


void SpeedControl::SetParameters(std::string thrust_map, double max_thrust,
  bool use_thrust_map_only) {
  if (thrust_map != "") {
    bool map_ok = m_thrust_map.injestMapString(thrust_map);
    if (!map_ok) {
      if (DEBUG)
        MOOSTrace("Speed Control: Error in Thrust Map");
      m_thrust_map_set = true;
    }
    m_use_thrust_map_only = use_thrust_map_only;
  }

  m_max_thrust = max_thrust;
}

bool SpeedControl::SpeedHistInfo(double time_range, double &slope,
                                 double &average) {
  if (time_range <= 0 || m_speed_hist.size() <= 1) {
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
    if (DEBUG)
      MOOSTrace("Speed Slope: %.2f  ", slope);
  }

  return valid_history;
}

double SpeedControl::TimeAtHeading(double allowable_range) {
  if (m_speed_hist.size() <= 1) {
    return 0;
  }
  double current_heading = m_speed_hist.front().m_heading;
  double oldest_time = m_speed_hist.front().m_time;

  std::list<SpeedRecord>::iterator record;
  for(record = m_speed_hist.begin(); record != m_speed_hist.end(); record++) {
    oldest_time = record->m_time;
    if (HeadingAbsDiff(record->m_heading, current_heading) > allowable_range) {
      break;
    }
  }

  return m_speed_hist.front().m_time - oldest_time;
}

// int SpeedControl::BinnedHeading(double heading) {
//   int binned = int(std::round(angle360(heading) / ANGLE_BINS));
//   // This is the case for 360-ANGLE_BINS/2
//   if (binned >= m_direction_average.size())
//     binned = 0;

//   return binned;
// }

std::string SpeedControl::AppCastMessage() {
  std::stringstream message;
  message << "Speed Control Enabled\n";
  message << "\nSpeed Averages:\n";

  message << m_current_estimate.AppCastMessage();

  message << "\nControl Adjust State: " << m_adjustment_state << std::endl;

  return message.str();
}

double SpeedControl::HeadingAbsDiff(double heading1, double heading2) {
  return fabs(angle180(angle180(heading1) - angle180(heading2)));
}

void SpeedControl::GetVarInfo(double * vars) {
  vars[0] = double(m_adjustment_state);
}
