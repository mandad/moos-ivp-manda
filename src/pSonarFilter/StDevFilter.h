/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: StDevFilter.h                                   */
/*    DATE: 2015-05-10                                      */
/************************************************************/


#ifndef MarineMRAS_StDevFilter_HEADER
#define MarineMRAS_StDevFilter_HEADER

#include <deque>
#include <vector>

class StDevFilter
{
  public:
    StDevFilter() {};
    StDevFilter(double length, double stdevs, double sample_T);
    ~StDevFilter() {};

    /**
     * Ingests a new value to filter
     * @param value The value to filter
     * @return The latest filtered value
     */
    bool IngestValue(double depth);

    /**
     * Gets the latest filtered value
     * @return The latest filtered value
     */
    double FilteredValue() { return m_filtered_depth; }

    void UpdateStats(double value);

  protected:
    std::deque<double> m_signal_in;
    std::deque<double> m_signal_out;

    double m_std_limit;
    double m_length;
    double m_a;
    double m_mean;
    double m_meansq;
    double m_stdev;
    double m_filtered_depth;

    bool m_first_value;

};

#endif
