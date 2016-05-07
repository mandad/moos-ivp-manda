/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SignalFilter.cpp                                */
/*    DATE: 2015-05-06                                      */
/************************************************************/

#include <cmath>
#include "SignalFilter.h"

SignalFilter::SignalFilter(double fc, double sample_T) : m_signal_in{0,0,0},
      m_signal_out{0,0,0} {
  // Determine the coefficients of the filter
  double wc = 2 * M_PI * fc;
  double c = cos(wc * sample_T * 0.5)/sin(wc * sample_T * 0.5);

  // Second order butterworth low pass
  // Note that this will not have linear phase shift, but its a bit easier for
  // now
  double c_sq = c * c;
  double sq_2c = sqrt(2)*c;
  m_n = {1, 2, 1};
  m_d = {c_sq + sq_2c + 1, -2 * (c_sq - 1), c_sq - sq_2c + 1};
  double factor = 1/m_d[0];
  // Assumes n and d are the same size
  for (int i = 0; i < m_n.size(); i++) {
    m_n[i] *= factor;
    m_d[i] *= factor;
  }
}

double SignalFilter::IngestValue(double value) {
  m_signal_in.push_front(value);
  m_signal_out.push_front(m_n[0] * value + m_n[1] * m_signal_in[1]
    + m_n[2] * m_signal_in[2] - m_d[1] * m_signal_out[0] - m_d[2] * m_signal_out[1]);

  m_signal_out.pop_back();
  m_signal_in.pop_back();

  return m_signal_out[0];
}

double SignalFilter::FilteredValue() {
  return m_signal_out.front();
}
