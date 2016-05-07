/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SignalFilter.h                                  */
/*    DATE: 2015-05-06                                      */
/************************************************************/


#ifndef MarineMRAS_SignalFilter_HEADER
#define MarineMRAS_SignalFilter_HEADER

#include <deque>
#include <vector>

class SignalFilter
{
  public:
    SignalFilter() {};
    SignalFilter(double fc, double sample_T);
    ~SignalFilter() {};

    /**
     * Ingests a new value to filter
     * @param value The value to filter
     * @return The latest filtered value
     */
    double IngestValue(double value);

    /**
     * Gets the latest filtered value
     * @return The latest filtered value
     */
    double FilteredValue();

  protected:
    std::deque<double> m_signal_in;
    std::deque<double> m_signal_out;

    double m_sample_T;
    double m_fc;
    // Coefficients for transfer function
    std::vector<double> m_n;
    std::vector<double> m_d;

};

#endif
