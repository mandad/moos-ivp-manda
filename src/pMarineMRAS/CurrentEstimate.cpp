/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.cpp                             */
/*    DATE: 2016-02-12                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "CurrentEstimate.h"
#include "AngleUtils.h"
#include <cmath>

CurrentEstimate::CurrentEstimate(double bin_width, double save_time) : 
    m_bin_width{bin_width}, m_save_time{save_time}, m_full_hist(save_time, 1000) {
    // Not sure if this properly sets m_history[0]
    for (int direction = 0; direction < int(std::round(360 / bin_width)); 
       direction++) {
        //MOOSTrace("Speed Control: Setting Direction = %i\n", direction);
        m_history.push_back(CurrentRecord(save_time, 100));
  }
}

bool CurrentEstimate::SaveHistory(SpeedInfoRecord record) {
    int heading_bin = BinnedHeading(record.heading);
    m_history[heading_bin].SaveRecord(record);
    m_full_hist.SaveRecord(record);

    return true;
}

bool CurrentEstimate::GetEstimate(double &mag, double &heading) {
    // Use COG estimate as default, then max binned direction
    if (m_full_hist.GetAverageCurrent(mag, heading)) {
        return true;
    } else {
        double max_current = 0;
        int max_dir = 0;
        int this_dir = 0;
        std::vector<CurrentRecord>::iterator record;
        for (record = m_history.begin(); record != m_history.end(); record++) {
            double diff = record->GetAverageSpeedDiff();
            if (diff > max_current) {
                max_current = diff;
                max_dir = this_dir;
            }
            this_dir++;
        }
        // Could refine this by seeing which bin is second highest, where 
        // smallest current is (if not 180 off)
        if (max_current > 0) {
            mag = max_current;
            heading = double(max_dir) * m_bin_width;
            return true;
        }
    }
    return false;
}

double CurrentEstimate::GetSpeedDiff(double heading) {
    int heading_bin = BinnedHeading(heading);
    double current_mag, current_heading;
    if (m_history[heading_bin].NumRecords() > 0) {
        return m_history[heading_bin].GetAverageSpeedDiff();
    } else if (GetEstimate(current_mag, current_heading)) {
        return speedInHeading(current_heading, current_mag, heading);
    }
    // Don't have any data to give an estimate yet
    return 0;
}

int CurrentEstimate::BinnedHeading(double heading) {
  int binned = int(std::round(angle360(heading) / m_bin_width));
  // This is the case for 360-ANGLE_BINS/2
  if (binned >= m_history.size())
    binned = 0;

  return binned;
}

std::string CurrentEstimate::AppCastMessage() {
  std::stringstream message;
  for (int direction = 0; direction < m_history.size(); direction++) {
    message << direction * m_bin_width << ": " << 
        m_history[direction].GetAverageSpeedDiff() << std::endl;
  }
  double mag, direction;
  if (GetEstimate(mag, direction)) {
      message << std::endl << "Current Estimate: " << mag << "< " << direction 
        << std::endl;
  }

  return message.str();
}