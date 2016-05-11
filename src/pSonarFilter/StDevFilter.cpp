/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: StDevFilter.cpp                                */
/*    DATE: 2015-05-10                                      */
/************************************************************/

#include "StDevFilter.h"
#include <cmath>
#include <iostream>

#define DEBUG false

StDevFilter::StDevFilter(double length, double stdevs, double sample_T) :
      m_first_value{true}, m_stdev{0}, m_filtered_depth{-1} {
  m_std_limit = stdevs;
  m_a = 1 - std::exp(-2 * M_PI * sample_T / length);
}

bool StDevFilter::IngestValue(double depth) {
  bool retval = false;
  UpdateStats(depth);
  #if DEBUG
  std::cout << "Ingesting Value: " << depth << std::endl;
  #endif

  if (fabs(m_mean - depth) <= m_stdev * m_std_limit) {
    m_filtered_depth = depth;
    retval = true;
  }
  #if DEBUG
  std::cout << "Filtered depth: " << m_filtered_depth
    << " Mean depth: " << m_mean
    << " Stdev depth: " << m_stdev
    << " a: " << m_a << std::endl;
  #endif
  return retval;
  /* Old Method
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
  */
}

void StDevFilter::UpdateStats(double value) {

  if (m_first_value) {
    m_mean = value;
    m_meansq = value * value;
    m_first_value = false;
  }

  // Based on http://dsp.stackexchange.com/questions/811/determining-the-mean-and-standard-deviation-in-real-time
  // Exponentially weighted stdev and mean

  // update the estimate of the mean and the mean square:
  m_mean = (1-m_a)*m_mean + m_a*value;
  m_meansq = (1-m_a)*m_meansq + m_a*(value*value);

  // calculate the estimate of the variance:
  double var = m_meansq - m_mean * m_mean;

  // and, if you want standard deviation:
  m_stdev = std::sqrt(var);

}
