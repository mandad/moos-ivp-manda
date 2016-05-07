/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentRecord.cpp                               */
/*    DATE: 2016-02-16                                      */
/************************************************************/

#include "CurrentRecord.h"
#include <cmath>

#define MIN_COG_RECORDS 20

CurrentRecord::CurrentRecord(double save_time, int max_records) :
    m_save_time{save_time}, m_max_records{max_records} {
}

bool CurrentRecord::SaveRecord(SpeedInfoRecord record) {
    m_record_hist.push_front(record);

    if (record.valid_cog) {
        std::complex<double> record_diff = VectorDiff(record);
        m_vector_hist.push_front(record_diff);
    }

    while (m_record_hist.size() > m_max_records ||
           record.time - m_record_hist.back().time > m_save_time) {
        if (m_record_hist.back().valid_cog) {
            m_vector_hist.pop_back();
        }
        m_record_hist.pop_back();
    }
    return true;
}

// The averages are calculated on demand instead of at insertion since they are
// only called when switching headings for the vehicle (fairly long period)
bool CurrentRecord::GetAverageCurrent(double &mag, double &heading) {
    if (m_vector_hist.size() < MIN_COG_RECORDS)
        return false;
    std::list<std::complex<double>>::iterator record;
    double sum_y = 0;
    double sum_x = 0;
    // This has a potential for overflow
    for (record = m_vector_hist.begin(); record != m_vector_hist.end(); record++) {
        sum_x += record->imag();
        sum_y += record->real();
    }

    double num_records(m_vector_hist.size());
    std::complex<double> avg_current(sum_y / num_records, sum_x / num_records);
    mag = std::abs(avg_current);
    heading = radToHeading(std::arg(avg_current));
    return true;
}

double CurrentRecord::GetAverageSpeedDiff() {
    if (m_record_hist.size() > 0) {
        std::list<SpeedInfoRecord>::iterator record;
        double sum = 0;
        for (record = m_record_hist.begin(); record != m_record_hist.end(); record++) {
            sum += record->speed_over_ground - record->speed_estimate;
        }
        return sum / double(m_record_hist.size());
    } else {
        return 0;
    }
}

int CurrentRecord::NumRecords() {
    return m_record_hist.size();
}

std::complex<double> CurrentRecord::VectorDiff(SpeedInfoRecord record) {
    double speed_through_water =  record.speed_estimate;
    if (record.valid_stw) {
      speed_through_water = record.speed_through_water;
    }
    std::complex<double> expected_vector(std::polar(speed_through_water,
        headingToRadians(record.heading)));
    if (record.valid_cog) {
        double cog_rad = headingToRadians(record.course_over_ground);
        std::complex<double> cog_vector(std::polar(record.speed_over_ground,
            cog_rad));
        return cog_vector - expected_vector;
    } else {
        std::complex<double> actual_vector(std::polar(record.speed_over_ground,
            headingToRadians(record.heading)));
        return actual_vector - expected_vector;
    }
}
