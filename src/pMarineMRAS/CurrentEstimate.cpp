/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.cpp                            */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "SpeedControl.h"
#include "AngleUtils.h"
#include <cmath>

CurrentEstimate::CurrentEstimate(double bin_width) : m_bin_width{bin_width} {

}

int CurrentEstimate::BinnedHeading(double heading) {
  int binned = int(std::round(angle360(heading) / m_bin_width));
  // This is the case for 360-ANGLE_BINS/2
  if (binned >= m_direction_average.size())
    binned = 0;

  return binned;
}