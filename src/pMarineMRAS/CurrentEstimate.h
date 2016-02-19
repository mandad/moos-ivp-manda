/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.h                               */
/*    DATE: 2016-02-12                                      */
/************************************************************/

#ifndef MarineMRAS_CurrentEstimate_HEADER
#define MarineMRAS_CurrentEstimate_HEADER

#include <utility>
#include <vector>
#include "CurrentRecord.h"

class CurrentEstimate
{

public:
    CurrentEstimate(double bin_width=20, double save_time=3600);
    ~CurrentEstimate() {}  
    double GetSpeedDiff(double heading);
    bool GetEstimate(double &mag, double &heading);
    bool SaveHistory(SpeedInfoRecord record);
    std::string AppCastMessage();

private:
    //Functions
    int BinnedHeading(double heading);
    void InitRecords();
    
    //State variables
    
    // std::map<int, std::pair<double, int>> m_direction_average;
    // double m_time_at_speed;

    //Configuration variables
    double m_bin_width;
    double m_save_time;

    std::vector<CurrentRecord> m_history;
    CurrentRecord m_full_hist;

};

#endif