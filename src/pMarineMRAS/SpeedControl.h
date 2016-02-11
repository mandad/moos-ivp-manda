/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SpeedControl.h                                  */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#ifndef MarineMRAS_SpeedControl_HEADER
#define MarineMRAS_SpeedControl_HEADER

#include "ThrustMap.h"
#include "CurrentEstimate.h"
#include <utility>

class SpeedControl
{

public:
    SpeedControl();
    ~SpeedControl() {}

    double Run(double desired_speed, double speed, double heading, 
               double desired_heading, double time, bool turning);
    void SetParameters(std::string thrust_map, double max_thrust, 
        bool use_thrust_map_only);
    std::string AppCastMessage();
    void GetVarInfo(double * vars);

private:
    //Functions
    void InitControls();
    bool SpeedHistInfo(double time_range, double &slope, double &average);
    double TimeAtHeading(double allowable_range);
    int BinnedHeading(double heading);
    double HeadingAbsDiff(double heading1, double heading2);

    //State variables
    bool m_first_run;
    bool m_thrust_map_set;
    bool m_use_thrust_map_only;
    double m_thrust_output;
    double m_desired_speed;
    double m_previous_desired_speed;
    double m_previous_desired_heading;
    bool m_has_adjust;
    double m_thrust_change_time;
    double m_previous_time;
    double m_initial_speed;
    double m_turning_time;
    double m_prev_time_at_heading;
    bool m_turn_began;
    bool m_turn_finished;
    int m_adjustment_state;

    std::map<int, std::pair<double, int>> m_direction_average;
    double m_time_at_speed;

    //Configuration variables
    ThrustMap m_thrust_map;
    double m_max_thrust;


    struct SpeedRecord {
        SpeedRecord(double desired_speed, double speed, double heading, 
                    double time) : m_desired_speed{desired_speed}, m_speed{speed},
                    m_heading{heading}, m_time{time} {}

        double m_desired_speed;
        double m_speed;
        double m_heading;
        double m_time;
    };
    std::list<SpeedRecord> m_speed_hist;

};

#endif